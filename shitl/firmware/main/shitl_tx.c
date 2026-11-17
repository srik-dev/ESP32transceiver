/*
 * shitl_tx.c — ESP32-SHITL-Tx: a network-controlled, timed 2.4 GHz transmitter.
 * Targets: esp32c6 (b/g/n/ax + BLE 5) and esp32s3 (b/g/n + BLE 5).
 *
 * The sibling of ESP32-Tx, with the same waveforms and the same command names,
 * but driven over WiFi instead of a USB cable:
 *
 *   - The board joins a control network as a station and waits for commands
 *     (HTTP, see net.c). The USB serial console accepts the same commands.
 *   - Nothing is on air until a job is started:  run <seconds> [delay-seconds]
 *     The board acknowledges, leaves the control network, transmits the
 *     configured waveform for exactly that long, then rejoins and reports.
 *
 * Why it leaves the network: there is one 2.4 GHz radio. Injecting on an
 * arbitrary channel, or holding a BLE test carrier, cannot be time-shared with
 * an association to an access point without chopping the waveform. So a job
 * owns the radio outright and the timer is what brings the board back.
 * While a job runs the board is unreachable over the air; `stop` on the serial
 * console aborts it, and a power cycle always returns it to idle-and-online
 * (jobs are never saved).
 *
 * Commands (settings are saved to NVS; all are accepted over serial and HTTP):
 *     mode  <wifi|ble>     what a job transmits (no reboot)
 *     ch    <1-14>         WiFi channel
 *     rate  <name>         1m 2m 5m 11m | 6m..54m | mcs0..mcs7 | he-mcs0..he-mcs9 (C6)
 *     pwr   <2-20>         WiFi TX power, dBm
 *     duty  <0-100>        on-air percent of a 100 ms window (WiFi, BLE DTM)
 *     ble / phy / bch / advch / itvl / bpwr / pat / len     see ble_tx.c
 *     led   <name|#rrggbb> status LED colour tag
 *     wifi  <ssid> [pass]  control network credentials  (wifi clear to forget)
 *     run   <secs> [delay] transmit for secs (0.1-3600), starting after delay
 *     stop                 cancel a pending job / abort a running one
 *     status               one STATUS line of key=value fields
 */

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_mac.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_app_desc.h"
#include "esp_ota_ops.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_console.h"
#include "led_strip.h"
#include "soc/soc_caps.h"
#include "sdkconfig.h"

#include "app.h"

#ifndef DEFAULT_CHANNEL
#define DEFAULT_CHANNEL 6
#endif

/* WS2812 status LED: GPIO8 on ESP32-C6-DevKitC-1, GPIO48 on most S3 boards. */
#ifndef LED_GPIO
#if CONFIG_IDF_TARGET_ESP32S3
#define LED_GPIO 48
#else
#define LED_GPIO 8
#endif
#endif
#define LED_LVL 48

#define RUN_MIN_MS    100
#define RUN_MAX_MS    3600000
#define DELAY_MAX_MS  3600000
#define LEAD_MS       300      /* lets the HTTP reply leave before the link drops */

static const char *TAG = "shitl-tx";

enum { MODE_WIFI = 0, MODE_BLE = 1 };
static volatile int g_mode      = MODE_WIFI;
static volatile int g_channel   = DEFAULT_CHANNEL;
volatile int        g_duty      = 100;
static volatile int g_txpwr_dbm = 20;

typedef struct {
    const char     *name;
    const char     *std;    /* "b" / "g" / "n" / "ax" */
    wifi_phy_mode_t mode;
    wifi_phy_rate_t rate;
} rate_entry_t;

static const rate_entry_t RATES[] = {
    {"1m",  "b", WIFI_PHY_MODE_11B, WIFI_PHY_RATE_1M_L},
    {"2m",  "b", WIFI_PHY_MODE_11B, WIFI_PHY_RATE_2M_L},
    {"5m",  "b", WIFI_PHY_MODE_11B, WIFI_PHY_RATE_5M_L},
    {"11m", "b", WIFI_PHY_MODE_11B, WIFI_PHY_RATE_11M_L},
    {"6m",  "g", WIFI_PHY_MODE_11G, WIFI_PHY_RATE_6M},
    {"9m",  "g", WIFI_PHY_MODE_11G, WIFI_PHY_RATE_9M},
    {"12m", "g", WIFI_PHY_MODE_11G, WIFI_PHY_RATE_12M},
    {"18m", "g", WIFI_PHY_MODE_11G, WIFI_PHY_RATE_18M},
    {"24m", "g", WIFI_PHY_MODE_11G, WIFI_PHY_RATE_24M},
    {"36m", "g", WIFI_PHY_MODE_11G, WIFI_PHY_RATE_36M},
    {"48m", "g", WIFI_PHY_MODE_11G, WIFI_PHY_RATE_48M},
    {"54m", "g", WIFI_PHY_MODE_11G, WIFI_PHY_RATE_54M},
    {"mcs0", "n", WIFI_PHY_MODE_HT20, WIFI_PHY_RATE_MCS0_LGI},
    {"mcs1", "n", WIFI_PHY_MODE_HT20, WIFI_PHY_RATE_MCS1_LGI},
    {"mcs2", "n", WIFI_PHY_MODE_HT20, WIFI_PHY_RATE_MCS2_LGI},
    {"mcs3", "n", WIFI_PHY_MODE_HT20, WIFI_PHY_RATE_MCS3_LGI},
    {"mcs4", "n", WIFI_PHY_MODE_HT20, WIFI_PHY_RATE_MCS4_LGI},
    {"mcs5", "n", WIFI_PHY_MODE_HT20, WIFI_PHY_RATE_MCS5_LGI},
    {"mcs6", "n", WIFI_PHY_MODE_HT20, WIFI_PHY_RATE_MCS6_LGI},
    {"mcs7", "n", WIFI_PHY_MODE_HT20, WIFI_PHY_RATE_MCS7_LGI},
#if SOC_WIFI_HE_SUPPORT
    {"he-mcs0", "ax", WIFI_PHY_MODE_HE20, WIFI_PHY_RATE_MCS0_LGI},
    {"he-mcs1", "ax", WIFI_PHY_MODE_HE20, WIFI_PHY_RATE_MCS1_LGI},
    {"he-mcs2", "ax", WIFI_PHY_MODE_HE20, WIFI_PHY_RATE_MCS2_LGI},
    {"he-mcs3", "ax", WIFI_PHY_MODE_HE20, WIFI_PHY_RATE_MCS3_LGI},
    {"he-mcs4", "ax", WIFI_PHY_MODE_HE20, WIFI_PHY_RATE_MCS4_LGI},
    {"he-mcs5", "ax", WIFI_PHY_MODE_HE20, WIFI_PHY_RATE_MCS5_LGI},
    {"he-mcs6", "ax", WIFI_PHY_MODE_HE20, WIFI_PHY_RATE_MCS6_LGI},
    {"he-mcs7", "ax", WIFI_PHY_MODE_HE20, WIFI_PHY_RATE_MCS7_LGI},
    {"he-mcs8", "ax", WIFI_PHY_MODE_HE20, WIFI_PHY_RATE_MCS8_LGI},
    {"he-mcs9", "ax", WIFI_PHY_MODE_HE20, WIFI_PHY_RATE_MCS9_LGI},
#endif
};
#define N_RATES (sizeof(RATES) / sizeof(RATES[0]))
#define DEFAULT_RATE_IDX 4   /* "6m" */

static volatile int       g_rate_idx = DEFAULT_RATE_IDX;
static volatile esp_err_t g_rate_err = ESP_OK;

static volatile uint64_t g_frames_ok  = 0;
static volatile uint64_t g_frames_err = 0;
static volatile uint64_t g_tx_bp      = 0;

static uint8_t s_mac[6];
static uint8_t s_frame[256];
static size_t  s_frame_len;

static led_strip_handle_t s_led = NULL;
static char    g_led_name[12] = "off";
static uint8_t g_led_rgb[3] = {0, 0, 0};

/* ---- the job ---- */
typedef enum { JOB_IDLE, JOB_PENDING, JOB_ACTIVE } job_state_t;
static volatile job_state_t s_job = JOB_IDLE;
static volatile bool     s_cancel = false;
static volatile bool     s_ota_lock = false;
static volatile int      s_job_ms = 0;          /* requested duration        */
static volatile int64_t  s_job_start_us = 0;
static volatile int64_t  s_job_end_us = 0;
static volatile uint32_t s_job_n = 0;           /* jobs finished since boot  */
static const char *volatile s_last = "none";    /* none|done|aborted|cancelled|error */
static volatile int      s_last_ms = 0;
static volatile uint64_t s_last_frames = 0;
static const char *volatile s_last_mode = "-";
static TaskHandle_t      s_job_task = NULL;

/* ------------------------------------------------------------ persistence --- */

void cfg_save(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_u8 (h, "ch",   (uint8_t)g_channel);
    nvs_set_u8 (h, "duty", (uint8_t)g_duty);
    nvs_set_u8 (h, "pwr",  (uint8_t)g_txpwr_dbm);
    nvs_set_u8 (h, "mode", (uint8_t)g_mode);
    ble_cfg_save(h);
    nvs_set_str(h, "rate", RATES[g_rate_idx].name);
    nvs_set_str(h, "led",  g_led_name);
    nvs_set_blob(h, "led_rgb", g_led_rgb, sizeof(g_led_rgb));
    esp_err_t e = nvs_commit(h);
    nvs_close(h);
    if (e != ESP_OK) ESP_LOGW(TAG, "settings save failed: %s", esp_err_to_name(e));
}

static void cfg_load(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) return;
    uint8_t v;
    if (nvs_get_u8(h, "ch",   &v) == ESP_OK && v >= 1 && v <= 14)  g_channel   = v;
    if (nvs_get_u8(h, "duty", &v) == ESP_OK && v <= 100)           g_duty      = v;
    if (nvs_get_u8(h, "pwr",  &v) == ESP_OK && v >= 2 && v <= 20)  g_txpwr_dbm = v;
    if (nvs_get_u8(h, "mode", &v) == ESP_OK && v <= MODE_BLE)      g_mode      = v;
    ble_cfg_load(h);
    char name[16];
    size_t len = sizeof(name);
    if (nvs_get_str(h, "rate", name, &len) == ESP_OK) {
        for (size_t i = 0; i < N_RATES; i++) {
            if (strcmp(name, RATES[i].name) == 0) { g_rate_idx = (int)i; break; }
        }
    }
    len = sizeof(g_led_name);
    if (nvs_get_str(h, "led", g_led_name, &len) != ESP_OK) strcpy(g_led_name, "off");
    len = sizeof(g_led_rgb);
    if (nvs_get_blob(h, "led_rgb", g_led_rgb, &len) != ESP_OK) memset(g_led_rgb, 0, 3);
    nvs_close(h);
}

/* ----------------------------------------------------------------- beacon --- */
/* A standards-conformant beacon, so any scanner lists the board as an ordinary
 * access point: SSID BEESDR-TX-chNN, BSSID = the board's MAC. IEs in 802.11
 * order: SSID, Supported Rates, DS, TIM, ERP, Extended Rates, then HT (n, ax)
 * and HE (ax) capability/operation. Identical to ESP32-Tx. */
#define TSF_OFFSET 24

static void build_beacon(int ch, int rate_idx)
{
    const char *std = RATES[rate_idx].std;
    bool ht = (strcmp(std, "n") == 0) || (strcmp(std, "ax") == 0);
    bool he = (strcmp(std, "ax") == 0);
    uint8_t *p = s_frame;

    *p++ = 0x80; *p++ = 0x00;                 /* FC: mgmt / beacon            */
    *p++ = 0x00; *p++ = 0x00;                 /* duration                     */
    memset(p, 0xff, 6); p += 6;               /* DA = broadcast               */
    memcpy(p, s_mac, 6); p += 6;              /* SA                           */
    memcpy(p, s_mac, 6); p += 6;              /* BSSID                        */
    *p++ = 0x00; *p++ = 0x00;                 /* seq (filled by HW)           */

    memset(p, 0, 8); p += 8;                  /* timestamp (TSF_OFFSET)       */
    *p++ = 0x64; *p++ = 0x00;                 /* beacon interval = 100 TU     */
    *p++ = 0x01; *p++ = 0x04;                 /* capability: ESS + short slot */

    char ssid[24];
    int sl = snprintf(ssid, sizeof(ssid), "BEESDR-TX-ch%02d", ch);
    *p++ = 0x00; *p++ = (uint8_t)sl;
    memcpy(p, ssid, sl); p += sl;

    static const uint8_t rates[] = {          /* 1,2,5.5,11 (basic) 6,9,12,18 */
        0x01, 0x08, 0x82, 0x84, 0x8b, 0x96, 0x0c, 0x12, 0x18, 0x24
    };
    memcpy(p, rates, sizeof(rates)); p += sizeof(rates);

    *p++ = 0x03; *p++ = 0x01; *p++ = (uint8_t)ch;                    /* DS    */
    *p++ = 0x05; *p++ = 0x04; *p++ = 0x00; *p++ = 0x01; *p++ = 0x00; *p++ = 0x00;  /* TIM */
    *p++ = 0x2a; *p++ = 0x01; *p++ = 0x00;                           /* ERP   */

    static const uint8_t xrates[] = { 0x32, 0x04, 0x30, 0x48, 0x60, 0x6c };
    memcpy(p, xrates, sizeof(xrates)); p += sizeof(xrates);

    if (ht) {
        *p++ = 0x2d; *p++ = 26;               /* HT Capabilities: 20 MHz, MCS 0-7 */
        *p++ = 0x0c; *p++ = 0x00;
        *p++ = 0x00;
        *p++ = 0xff; memset(p, 0, 15); p += 15;
        memset(p, 0, 2 + 4 + 1); p += 7;

        *p++ = 0x3d; *p++ = 22;               /* HT Operation: primary = ch   */
        *p++ = (uint8_t)ch;
        memset(p, 0, 21); p += 21;
    }
    if (he) {
        *p++ = 0xff; *p++ = 1 + 6 + 11 + 4; *p++ = 35;   /* HE Capabilities   */
        memset(p, 0, 6);  p += 6;
        memset(p, 0, 11); p += 11;
        *p++ = 0xfd; *p++ = 0xff;
        *p++ = 0xfd; *p++ = 0xff;

        *p++ = 0xff; *p++ = 1 + 3 + 1 + 2; *p++ = 36;    /* HE Operation      */
        *p++ = 0x00; *p++ = 0x00; *p++ = 0x00;
        *p++ = (uint8_t)((s_mac[5] % 63) + 1);           /* BSS colour        */
        *p++ = 0xfc; *p++ = 0xff;
    }

    s_frame_len = (size_t)(p - s_frame);
}

/* ------------------------------------------------------------------- LED --- */

static void led_set(uint8_t r, uint8_t g, uint8_t b)
{
    if (!s_led) return;
    led_strip_set_pixel(s_led, 0, r, g, b);
    led_strip_refresh(s_led);
}

static void led_init(void)
{
    led_strip_config_t sc = {
        .strip_gpio_num = LED_GPIO,
        .max_leds = 1,
        .led_pixel_format = LED_PIXEL_FORMAT_GRB,
        .led_model = LED_MODEL_WS2812,
        .flags.invert_out = false,
    };
    led_strip_rmt_config_t rc = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,
        .mem_block_symbols = 64,
        .flags.with_dma = false,
    };
    if (led_strip_new_rmt_device(&sc, &rc, &s_led) == ESP_OK) {
        led_set(g_led_rgb[0], g_led_rgb[1], g_led_rgb[2]);
    } else {
        s_led = NULL;
        ESP_LOGW(TAG, "no WS2812 on GPIO%d", LED_GPIO);
    }
}

/* -------------------------------------------------------- WiFi injection --- */

/* PHY mode + rate for injected frames. Do not esp_wifi_stop() around it (the
 * library then returns ESP_FAIL for every rate); try it live and fall back to
 * the documented init order only if that is refused. */
static esp_err_t apply_rate(int idx)
{
    wifi_tx_rate_config_t rc = {
        .phymode = RATES[idx].mode, .rate = RATES[idx].rate,
        .ersu = false, .dcm = false,
    };
    esp_err_t e = esp_wifi_config_80211_tx(WIFI_IF_STA, &rc);
    if (e == ESP_OK) return ESP_OK;
    ESP_LOGW(TAG, "live rate config refused (%s), re-initialising STA", esp_err_to_name(e));
    esp_wifi_stop();
    esp_wifi_set_mode(WIFI_MODE_NULL);
    esp_wifi_set_mode(WIFI_MODE_STA);
    e = esp_wifi_config_80211_tx(WIFI_IF_STA, &rc);
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_start());
    return e;
}

/* Inject beacons, duty-cycled in 100 ms windows, until end_us or cancel. The
 * station is disconnected for the whole call, so the channel is ours. */
static int wifi_tx_run(int64_t end_us)
{
    const int WINDOW_MS = 100;
    int ch = g_channel, pwr = g_txpwr_dbm, rate = g_rate_idx;
    int duty = g_duty;
    if (duty < 0)   duty = 0;
    if (duty > 100) duty = 100;

    g_rate_err = apply_rate(rate);
    if (g_rate_err != ESP_OK) ESP_LOGE(TAG, "rate %s rejected: %s", RATES[rate].name, esp_err_to_name(g_rate_err));
    esp_err_t e = esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "set_channel %d: %s", ch, esp_err_to_name(e));
        return -1;
    }
    build_beacon(ch, rate);
    esp_wifi_set_max_tx_power((int8_t)(pwr * 4));      /* 0.25 dBm units */
    ESP_LOGI(TAG, "wifi job: ch=%d rate=%s pwr=%d duty=%d", ch, RATES[rate].name, pwr, duty);

    int on_ms  = WINDOW_MS * duty / 100;
    int off_ms = WINDOW_MS - on_ms;

    while (!s_cancel && esp_timer_get_time() < end_us) {
        int64_t t_on = esp_timer_get_time() + (int64_t)on_ms * 1000;
        if (t_on > end_us) t_on = end_us;
        while (!s_cancel && esp_timer_get_time() < t_on) {
            uint64_t tsf = (uint64_t)esp_timer_get_time();
            for (int i = 0; i < 8; i++) s_frame[TSF_OFFSET + i] = (uint8_t)(tsf >> (8 * i));
            e = esp_wifi_80211_tx(WIFI_IF_STA, s_frame, s_frame_len, true);
            if (e == ESP_OK) {
                g_frames_ok++;
            } else if (e == ESP_ERR_NO_MEM) {
                g_tx_bp++;            /* TX buffers full: normal backpressure */
                vTaskDelay(1);
            } else {
                g_frames_err++;
            }
        }
        /* One yield per window keeps the idle task, the WDT and the console alive. */
        int64_t left_ms = (end_us - esp_timer_get_time()) / 1000;
        int nap = off_ms > 0 ? off_ms : 1;
        if (left_ms < nap) nap = left_ms > 0 ? (int)left_ms : 1;
        vTaskDelay(pdMS_TO_TICKS(nap));
    }
    return 0;
}

/* ------------------------------------------------------------ job runner --- */

bool job_cancelled(void) { return s_cancel; }
bool job_idle(void)      { return s_job == JOB_IDLE; }

bool job_lock_for_ota(bool lock)
{
    if (lock && s_job != JOB_IDLE) return false;
    s_ota_lock = lock;
    return true;
}

static void job_task(void *arg)
{
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (s_job != JOB_PENDING) continue;

        while (!s_cancel && esp_timer_get_time() < s_job_start_us) vTaskDelay(pdMS_TO_TICKS(10));
        if (s_cancel) {
            s_last = "cancelled"; s_last_ms = 0; s_last_frames = 0;
            s_job_n++;
            s_job = JOB_IDLE;
            continue;
        }

        int mode = g_mode;
        bool ble = (mode == MODE_BLE);
        s_last_mode = ble ? "ble" : "wifi";
        net_offline_begin(ble);
        int rc = ble ? ble_tx_up() : 0;       /* stack bring-up is not billed to the job */

        uint64_t f0 = g_frames_ok;
        int64_t t0 = esp_timer_get_time();
        s_job_end_us = t0 + (int64_t)s_job_ms * 1000;
        s_job = JOB_ACTIVE;
        if (rc == 0) rc = ble ? ble_tx_run(s_job_end_us) : wifi_tx_run(s_job_end_us);
        int64_t t1 = esp_timer_get_time();

        if (ble) ble_tx_down();
        net_offline_end(ble);
        s_last_ms = (int)((t1 - t0) / 1000);
        s_last_frames = g_frames_ok - f0;
        s_last = rc != 0 ? "error" : s_cancel ? "aborted" : "done";
        s_job_n++;
        s_job = JOB_IDLE;
        ESP_LOGI(TAG, "job %s: %d ms, %llu frames", s_last, s_last_ms, (unsigned long long)s_last_frames);
    }
}

/* --------------------------------------------------------------- commands --- */

static SemaphoreHandle_t s_mx;
static char  *s_out;
static size_t s_out_n, s_out_len;

void rpl(const char *fmt, ...)
{
    if (!s_out || s_out_len + 1 >= s_out_n) return;
    va_list ap;
    va_start(ap, fmt);
    int k = vsnprintf(s_out + s_out_len, s_out_n - s_out_len, fmt, ap);
    va_end(ap);
    if (k > 0) {
        s_out_len += (size_t)k;
        if (s_out_len > s_out_n - 1) s_out_len = s_out_n - 1;
    }
}

static int cmd_mode(int argc, char **argv)
{
    if (argc < 2) { rpl("ERR usage: mode <wifi|ble>\n"); return 1; }
    if      (strcmp(argv[1], "wifi") == 0) g_mode = MODE_WIFI;
    else if (strcmp(argv[1], "ble")  == 0) g_mode = MODE_BLE;
    else { rpl("ERR expected wifi|ble\n"); return 1; }
    cfg_save();
    rpl("OK mode=%s\n", argv[1]);
    return 0;
}

static int cmd_ch(int argc, char **argv)
{
    int v = argc >= 2 ? atoi(argv[1]) : 0;
    if (v < 1 || v > 14) { rpl("ERR channel out of range 1-14\n"); return 1; }
    g_channel = v;
    cfg_save();
    rpl("OK ch=%d\n", v);
    return 0;
}

static int cmd_duty(int argc, char **argv)
{
    char *end = NULL;
    long v = argc >= 2 ? strtol(argv[1], &end, 10) : -1;
    if (argc < 2 || *end != '\0' || v < 0 || v > 100) { rpl("ERR duty out of range 0-100\n"); return 1; }
    g_duty = (int)v;
    cfg_save();
    rpl("OK duty=%d\n", (int)v);
    return 0;
}

static int cmd_pwr(int argc, char **argv)
{
    int v = argc >= 2 ? atoi(argv[1]) : 0;
    if (v < 2 || v > 20) { rpl("ERR pwr out of range 2-20 dBm\n"); return 1; }
    g_txpwr_dbm = v;
    cfg_save();
    rpl("OK pwr=%d\n", v);
    return 0;
}

static int cmd_rate(int argc, char **argv)
{
    if (argc < 2) { rpl("ERR usage: rate <name>\n"); return 1; }
    for (size_t i = 0; i < N_RATES; i++) {
        if (strcmp(argv[1], RATES[i].name) == 0) {
            g_rate_idx = (int)i;
            cfg_save();
            rpl("OK rate=%s phy=%s\n", RATES[i].name, RATES[i].std);
            return 0;
        }
    }
    rpl("ERR unknown rate '%s'\n", argv[1]);
    return 1;
}

static int cmd_led(int argc, char **argv)
{
    if (argc < 2) { rpl("ERR usage: led <off|red|green|blue|yellow|cyan|magenta|white|orange|#rrggbb>\n"); return 1; }
    int r = 0, g = 0, b = 0;
    const int L = LED_LVL;
    if (argv[1][0] == '#') {
        char *end;
        if (strlen(argv[1]) != 7) { rpl("ERR bad hex colour '%s'\n", argv[1]); return 1; }
        unsigned v = (unsigned)strtoul(argv[1] + 1, &end, 16);
        if (*end != '\0') { rpl("ERR bad hex colour '%s'\n", argv[1]); return 1; }
        r = (int)(((v >> 16) & 0xff) * L + 254) / 255;
        g = (int)(((v >>  8) & 0xff) * L + 254) / 255;
        b = (int)(( v        & 0xff) * L + 254) / 255;
        snprintf(g_led_name, sizeof(g_led_name), "#%06x", v);
    } else {
        const struct { const char *n; int r, g, b; } c[] = {
            {"off",0,0,0}, {"red",L,0,0}, {"green",0,L,0}, {"blue",0,0,L},
            {"yellow",L,L,0}, {"cyan",0,L,L}, {"magenta",L,0,L},
            {"white",L,L,L}, {"orange",L,L/4,0},
        };
        int found = 0;
        for (size_t i = 0; i < sizeof(c)/sizeof(c[0]); i++) {
            if (strcmp(argv[1], c[i].n) == 0) {
                r = c[i].r; g = c[i].g; b = c[i].b; found = 1;
                snprintf(g_led_name, sizeof(g_led_name), "%s", c[i].n);
                break;
            }
        }
        if (!found) { rpl("ERR unknown color '%s'\n", argv[1]); return 1; }
    }
    bool changed = g_led_rgb[0] != r || g_led_rgb[1] != g || g_led_rgb[2] != b;
    g_led_rgb[0] = (uint8_t)r; g_led_rgb[1] = (uint8_t)g; g_led_rgb[2] = (uint8_t)b;
    led_set(g_led_rgb[0], g_led_rgb[1], g_led_rgb[2]);
    if (changed) cfg_save();
    rpl("OK led=%s\n", g_led_name);
    return 0;
}

static int cmd_wifi(int argc, char **argv)
{
    if (argc < 2) { rpl("OK wifi ssid=%s\n", net_ssid()[0] ? net_ssid() : "-"); return 0; }
    if (argc == 2 && strcmp(argv[1], "clear") == 0) {
        net_set_creds("", "");
        rpl("OK wifi cleared\n");
        return 0;
    }
    const char *psk = argc >= 3 ? argv[2] : "";
    size_t pl = strlen(psk);
    if (strlen(argv[1]) > 32) { rpl("ERR ssid longer than 32 bytes\n"); return 1; }
    if (pl > 63 || (pl > 0 && pl < 8)) { rpl("ERR password must be 8-63 characters (or omitted for an open network)\n"); return 1; }
    net_set_creds(argv[1], psk);
    rpl("OK wifi ssid=%s joining\n", argv[1]);
    return 0;
}

/* "12", "0.5", "2.25" -> milliseconds; -1 if not a number. */
static long parse_ms(const char *s)
{
    char *end;
    double v = strtod(s, &end);
    if (end == s || *end != '\0' || v < 0 || v > 1e7) return -1;
    return (long)(v * 1000.0 + 0.5);
}

static int cmd_run(int argc, char **argv)
{
    if (argc < 2) { rpl("ERR usage: run <seconds 0.1-3600> [delay-seconds 0-3600]\n"); return 1; }
    long ms = parse_ms(argv[1]);
    long dly = argc >= 3 ? parse_ms(argv[2]) : 0;
    if (ms < RUN_MIN_MS || ms > RUN_MAX_MS) { rpl("ERR duration out of range 0.1-3600 s\n"); return 1; }
    if (dly < 0 || dly > DELAY_MAX_MS) { rpl("ERR delay out of range 0-3600 s\n"); return 1; }
    if (s_ota_lock) { rpl("ERR busy: firmware update in progress\n"); return 1; }
    if (s_job != JOB_IDLE) { rpl("ERR busy: a job is already %s\n", s_job == JOB_PENDING ? "pending" : "running"); return 1; }
    if (dly < LEAD_MS) dly = LEAD_MS;
    s_cancel = false;
    s_job_ms = (int)ms;
    s_job_start_us = esp_timer_get_time() + (int64_t)dly * 1000;
    s_job = JOB_PENDING;
    xTaskNotifyGive(s_job_task);
    rpl("OK run ms=%ld delay_ms=%ld mode=%s\n", ms, dly, g_mode == MODE_BLE ? "ble" : "wifi");
    return 0;
}

static int cmd_stop(int argc, char **argv)
{
    if (s_job == JOB_IDLE) { rpl("OK stop idle\n"); return 0; }
    s_cancel = true;
    rpl("OK stop\n");
    return 0;
}

#define CLAMP(k, n) do { if ((k) < 0) (k) = 0; if ((size_t)(k) > (n) - 1) (k) = (int)(n) - 1; } while (0)

int status_snprint(char *buf, size_t n)
{
    const esp_app_desc_t *app = esp_app_get_description();
    const esp_partition_t *part = esp_ota_get_running_partition();
    job_state_t js = s_job;
    int64_t now = esp_timer_get_time();
    long in_ms = 0, left_ms = 0;
    if (js == JOB_PENDING) { in_ms = (long)((s_job_start_us - now) / 1000); if (in_ms < 0) in_ms = 0; }
    if (js == JOB_ACTIVE)  { left_ms = (long)((s_job_end_us - now) / 1000); if (left_ms < 0) left_ms = 0; }

    int k = snprintf(buf, n,
        "STATUS mac=%02x:%02x:%02x:%02x:%02x:%02x chip=" CONFIG_IDF_TARGET " fw=shitltx ver=%s part=%s"
        " mode=%s ch=%d duty=%d pwr=%ddBm rate=%s phy=%s rate_err=%s tx=%s led=%s"
        " frames_ok=%llu frames_err=%llu bp=%llu ssid=BEESDR-TX-ch%02d",
        s_mac[0], s_mac[1], s_mac[2], s_mac[3], s_mac[4], s_mac[5],
        app->version, part ? part->label : "?",
        g_mode == MODE_BLE ? "ble" : "wifi", g_channel, g_duty, g_txpwr_dbm,
        RATES[g_rate_idx].name, RATES[g_rate_idx].std,
        g_rate_err == ESP_OK ? "0" : esp_err_to_name(g_rate_err),
        js == JOB_ACTIVE ? "on" : "off", g_led_name,
        (unsigned long long)g_frames_ok, (unsigned long long)g_frames_err,
        (unsigned long long)g_tx_bp, g_channel);
    CLAMP(k, n);
    k += ble_status_snprint(buf + k, n - k); CLAMP(k, n);
    k += net_status_snprint(buf + k, n - k); CLAMP(k, n);
    k += snprintf(buf + k, n - k,
        " job=%s job_ms=%d job_in_ms=%ld job_left_ms=%ld job_n=%u last=%s last_mode=%s last_ms=%d last_frames=%llu up=%lld",
        js == JOB_IDLE ? "idle" : js == JOB_PENDING ? "pending" : "active",
        s_job_ms, in_ms, left_ms, (unsigned)s_job_n, s_last, s_last_mode, s_last_ms,
        (unsigned long long)s_last_frames, (long long)(now / 1000000));
    CLAMP(k, n);
    return k;
}

static int cmd_status(int argc, char **argv)
{
    if (s_out) {
        s_out_len += (size_t)status_snprint(s_out + s_out_len, s_out_n - s_out_len);
        rpl("\n");
    }
    return 0;
}

static const struct { const char *name; int (*fn)(int, char **); const char *help; } CMDS[] = {
    {"mode",   cmd_mode,   "what a job transmits: wifi | ble"},
    {"ch",     cmd_ch,     "WiFi channel 1-14"},
    {"rate",   cmd_rate,   "WiFi PHY rate / standard b,g,n,ax"},
    {"pwr",    cmd_pwr,    "WiFi TX power 2-20 dBm"},
    {"duty",   cmd_duty,   "on-air duty cycle 0-100 pct"},
    {"ble",    cmd_ble,    "BLE kind: adv | ext | dtm"},
    {"phy",    cmd_phy,    "BLE PHY 1m|2m|s8|s2 (ext, dtm)"},
    {"bch",    cmd_bch,    "BLE DTM RF channel 0-39"},
    {"advch",  cmd_advch,  "BLE advertising channels all|37,38,39"},
    {"itvl",   cmd_itvl,   "BLE advertising interval, ms"},
    {"bpwr",   cmd_bpwr,   "BLE TX power, dBm"},
    {"pat",    cmd_pat,    "BLE DTM payload pattern"},
    {"len",    cmd_len,    "BLE DTM payload / ext adv data bytes"},
    {"led",    cmd_led,    "status LED colour"},
    {"wifi",   cmd_wifi,   "control network: wifi <ssid> [password] | wifi clear"},
    {"run",    cmd_run,    "transmit: run <seconds> [delay-seconds]"},
    {"stop",   cmd_stop,   "cancel / abort the job"},
    {"status", cmd_status, "print the STATUS line"},
};
#define N_CMDS (sizeof(CMDS) / sizeof(CMDS[0]))

static int exec_argv(int argc, char **argv, char *out, size_t n)
{
    int rc = 1;
    out[0] = '\0';
    if (argc == 0) return 0;
    xSemaphoreTake(s_mx, portMAX_DELAY);
    s_out = out; s_out_n = n; s_out_len = 0;
    size_t i;
    for (i = 0; i < N_CMDS; i++) {
        if (strcmp(argv[0], CMDS[i].name) == 0) { rc = CMDS[i].fn(argc, argv); break; }
    }
    if (i == N_CMDS) rpl("ERR unknown command '%s'\n", argv[0]);
    s_out = NULL;
    xSemaphoreGive(s_mx);
    return rc;
}

int cmd_exec_line(char *line, char *out, size_t n)
{
    char *argv[8];
    size_t argc = esp_console_split_argv(line, argv, 8);
    return exec_argv((int)argc, argv, out, n);
}

/* Serial console: every command name is registered to this one entry point. */
static int con_cmd(int argc, char **argv)
{
    static char buf[1280];
    int rc = exec_argv(argc, argv, buf, sizeof(buf));
    fputs(buf, stdout);
    return rc;
}

/* ------------------------------------------------------------------- main --- */

void app_main(void)
{
    esp_err_t nv = nvs_flash_init();
    if (nv == ESP_ERR_NVS_NO_FREE_PAGES || nv == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nv = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nv);
    cfg_load();
    s_mx = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_read_mac(s_mac, ESP_MAC_WIFI_STA));
    led_init();

    xTaskCreate(job_task, "job", 4096, NULL, 5, &s_job_task);
    net_start();
    ble_tx_init();

    ESP_LOGI(TAG, "ESP32-SHITL-Tx " CONFIG_IDF_TARGET " %s up. mac=%02x:%02x:%02x:%02x:%02x:%02x",
             esp_app_get_description()->version,
             s_mac[0], s_mac[1], s_mac[2], s_mac[3], s_mac[4], s_mac[5]);

    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_cfg = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_cfg.prompt = "shitl-tx>";
    repl_cfg.max_cmdline_length = 256;
#if defined(CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG)
    esp_console_dev_usb_serial_jtag_config_t hw = ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_serial_jtag(&hw, &repl_cfg, &repl));
#else
    esp_console_dev_uart_config_t hw = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&hw, &repl_cfg, &repl));
#endif
    for (size_t i = 0; i < N_CMDS; i++) {
        const esp_console_cmd_t c = {.command = CMDS[i].name, .help = CMDS[i].help, .func = con_cmd};
        ESP_ERROR_CHECK(esp_console_cmd_register(&c));
    }
    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}
