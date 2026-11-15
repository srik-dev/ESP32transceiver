/*
 * beacon_tx.c — BEESDR ESP32 WiFi transmitter for SDR/AoA lab work.
 * Targets: esp32c6 (b/g/n/ax) and esp32s3 (b/g/n — no HE radio).
 *
 * Puts the chip into a raw-injection "transmitter mode": it sits on one 2.4 GHz
 * WiFi channel and continuously injects well-formed 802.11 beacon frames via
 * esp_wifi_80211_tx(). To the B210 this looks like real WiFi — bursty, 20 MHz
 * OFDM-shaped energy on a chosen channel — which is what the AoA / burst-gate
 * work downstream is tuned to see.
 *
 * Why beacons (management frames) and not data frames: esp_wifi_80211_tx()
 * rejects data frames whose addressing does not match the interface, but passes
 * arbitrary management frames. Beacons are the reliable, portable injection
 * primitive across IDF versions.
 *
 * Runtime control over the USB-Serial-JTAG console (same port as the log):
 *     ch    <1-14>     set channel   (SSID + DS-param IE track it)
 *     duty  <0-100>    on-air percent of a 100 ms window (bursty vs continuous)
 *     pwr   <2-20>     max TX power in dBm
 *     rate  <name>     PHY rate, which also selects the 802.11 standard:
 *                        b : 1m 2m 5m 11m            (DSSS/CCK)
 *                        g : 6m 9m 12m 18m 24m 36m 48m 54m (OFDM, 6m default)
 *                        n : mcs0..mcs7              (HT20)
 *                        ax: he-mcs0..he-mcs9        (HE20 SU, 1.6 us GI; C6 only)
 *     tx    <on|off>   enable / pause injection
 *     status           print current state + frame/error counters
 *     led   <name|#rrggbb>  status LED colour (named, or any hex colour;
 *                        hex is dimmed to the same safe brightness)
 *
 * Every setting above (plus the LED colour) is saved to NVS flash the moment it
 * is changed and restored at boot, so a board unplugged from the PC and powered
 * from any USB supply comes straight back up transmitting exactly what it was
 * last told to — standalone, no host needed. The PC/dashboard is only needed to
 * change the settings.
 *
 * One firmware image serves every board; boards are told apart by channel
 * (assigned per-board with tools/esp32tx_ctl.py) and by a MAC-derived BSSID.
 *
 * WiFi or Bluetooth LE — one radio, one at a time:
 *     mode  <wifi|ble>  select the radio personality. Saved, then the chip
 *                       reboots (~1 s) into it with only that stack running.
 * BLE mode (legacy / extended advertising, Coded PHY, DTM test carrier) and its
 * commands live in ble_tx.c; `tx`, `duty`, `led` and `status` serve both modes.
 *
 * BEESDR drone-RF R&D — ESP32-Tx.
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_mac.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_console.h"
#include "esp_rom_sys.h"      /* esp_rom_delay_us */
#include "led_strip.h"        /* addressable RGB status LED (WS2812) */
#include "soc/soc_caps.h"     /* SOC_WIFI_HE_SUPPORT */
#include "sdkconfig.h"

#include "ble_tx.h"

/* ---- Compile-time default channel (override at runtime with `ch`). The flash
 * helper also assigns a per-board channel over serial right after flashing, so
 * this is only the pre-assignment boot value. ---- */
#ifndef DEFAULT_CHANNEL
#define DEFAULT_CHANNEL 6
#endif

/* Addressable RGB LED (WS2812) pin. Used as a per-board colour tag so boards
 * are told apart at a glance; the dashboard sets it from a MAC->colour map.
 *   ESP32-C6-DevKitC-1 : GPIO8
 *   ESP32-S3-DevKitC-1 : GPIO48 (v1.0 and most clones; v1.1 moved it to GPIO38
 *                        — build with -DLED_GPIO=38 for those) */
#ifndef LED_GPIO
#if CONFIG_IDF_TARGET_ESP32S3
#define LED_GPIO 48
#else
#define LED_GPIO 8
#endif
#endif
#define LED_LVL 48   /* per-channel brightness 0-255; low to limit current */

static const char *TAG = "beesdr-tx";

/* ---- Shared TX config. Plain scalars written from the console task and read
 * by the TX task; word-sized and single-writer per field, so no lock needed for
 * this lab tool. ---- */
static volatile int      g_channel   = DEFAULT_CHANNEL;  /* 1..14              */
volatile int             g_duty      = 100;              /* 0..100 percent     */
static volatile int      g_txpwr_dbm = 20;               /* 2..20 dBm          */
volatile bool            g_tx_on     = true;

/* Radio personality. Read once at boot — changing it reboots (see cmd_mode). */
enum { MODE_WIFI = 0, MODE_BLE = 1 };
static int               g_mode      = MODE_WIFI;

/* PHY rate table. Each entry carries the PHY mode, because the mode — not the
 * rate code — is what decides the 802.11 standard on air: MCS0 is HT (11n)
 * under WIFI_PHY_MODE_HT20 and HE (11ax) under WIFI_PHY_MODE_HE20. */
typedef struct {
    const char     *name;   /* console name                     */
    const char     *std;    /* "b" / "g" / "n" / "ax"           */
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
#if SOC_WIFI_HE_SUPPORT   /* 11ax needs an HE radio: C6 yes, S3 no (b/g/n only) */
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
static volatile esp_err_t g_rate_err = ESP_OK;   /* last config_80211_tx result */

/* ---- Counters for `status`. ---- */
static volatile uint64_t g_frames_ok  = 0;
static volatile uint64_t g_frames_err = 0;
static volatile uint64_t g_tx_bp      = 0;   /* TX-buffer backpressure (normal) */

static uint8_t s_mac[6];
static uint8_t s_frame[256];
static size_t  s_frame_len;

static led_strip_handle_t s_led = NULL;
static char g_led_name[12] = "off";
static uint8_t g_led_rgb[3] = {0, 0, 0};

/* ------------------------------------------------------------ persistence --- */
/* Settings live in NVS namespace "beesdr". Written on every accepted console
 * command (changes are human-paced, so flash wear is a non-issue) and read once
 * at boot. The rate is stored by NAME, not table index, so reordering RATES[]
 * in a later firmware cannot silently map an old setting to a different rate. */
#define NVS_NS "beesdr"

void cfg_save(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_u8 (h, "ch",   (uint8_t)g_channel);
    nvs_set_u8 (h, "duty", (uint8_t)g_duty);
    nvs_set_u8 (h, "pwr",  (uint8_t)g_txpwr_dbm);
    nvs_set_u8 (h, "tx",   g_tx_on ? 1 : 0);
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
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) {
        ESP_LOGI(TAG, "no saved settings, using defaults");
        return;
    }
    uint8_t v;
    if (nvs_get_u8(h, "ch",   &v) == ESP_OK && v >= 1 && v <= 14)  g_channel   = v;
    if (nvs_get_u8(h, "duty", &v) == ESP_OK && v <= 100)           g_duty      = v;
    if (nvs_get_u8(h, "pwr",  &v) == ESP_OK && v >= 2 && v <= 20)  g_txpwr_dbm = v;
    if (nvs_get_u8(h, "tx",   &v) == ESP_OK)                       g_tx_on     = (v != 0);
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
    ESP_LOGI(TAG, "restored: mode=%s ch=%d duty=%d pwr=%d rate=%s tx=%s led=%s",
             g_mode == MODE_BLE ? "ble" : "wifi", g_channel, g_duty, g_txpwr_dbm, RATES[g_rate_idx].name,
             g_tx_on ? "on" : "off", g_led_name);
}

/* Build a standards-conformant beacon so any scanner (phone, laptop,
 * Wireshark, the SDR operator) lists the board as an ordinary access point:
 * SSID BEESDR-TX-chNN, BSSID = the board's own Espressif-OUI MAC.
 *
 * IEs, in the order 802.11 requires for a beacon:
 *   SSID, Supported Rates, DS Parameter Set, TIM (mandatory in beacons),
 *   ERP, Extended Supported Rates, then by PHY:
 *   n  -> + HT Capabilities, HT Operation
 *   ax -> + HT Capabilities, HT Operation, HE Capabilities, HE Operation
 *          (a 2.4 GHz HE BSS must also advertise HT)
 * The capability IEs are what make a scanner label the network Wi-Fi 4 / 6;
 * the PHY the frame is actually sent on does not, on its own. */
#define TSF_OFFSET 24   /* timestamp field, refreshed before every send */

static void build_beacon(int ch, int rate_idx)
{
    const char *std = RATES[rate_idx].std;
    bool ht = (strcmp(std, "n") == 0) || (strcmp(std, "ax") == 0);
    bool he = (strcmp(std, "ax") == 0);
    uint8_t *p = s_frame;

    *p++ = 0x80; *p++ = 0x00;                 /* FC: mgmt / beacon            */
    *p++ = 0x00; *p++ = 0x00;                 /* duration                     */
    memset(p, 0xff, 6); p += 6;               /* addr1 DA = broadcast         */
    memcpy(p, s_mac, 6); p += 6;              /* addr2 SA                     */
    memcpy(p, s_mac, 6); p += 6;              /* addr3 BSSID                  */
    *p++ = 0x00; *p++ = 0x00;                 /* seq (filled by HW, en_sys_seq) */

    memset(p, 0, 8); p += 8;                  /* timestamp (TSF_OFFSET)       */
    *p++ = 0x64; *p++ = 0x00;                 /* beacon interval = 100 TU     */
    *p++ = 0x01; *p++ = 0x04;                 /* capability: ESS + short slot */

    char ssid[24];
    int sl = snprintf(ssid, sizeof(ssid), "BEESDR-TX-ch%02d", ch);
    *p++ = 0x00; *p++ = (uint8_t)sl;          /* SSID                         */
    memcpy(p, ssid, sl); p += sl;

    static const uint8_t rates[] = {          /* Supported Rates: 1,2,5.5,11 (basic) 6,9,12,18 */
        0x01, 0x08, 0x82, 0x84, 0x8b, 0x96, 0x0c, 0x12, 0x18, 0x24
    };
    memcpy(p, rates, sizeof(rates)); p += sizeof(rates);

    *p++ = 0x03; *p++ = 0x01; *p++ = (uint8_t)ch;   /* DS Parameter Set       */

    /* TIM: DTIM count 0, DTIM period 1, bitmap control 0, empty bitmap */
    *p++ = 0x05; *p++ = 0x04; *p++ = 0x00; *p++ = 0x01; *p++ = 0x00; *p++ = 0x00;

    *p++ = 0x2a; *p++ = 0x01; *p++ = 0x00;          /* ERP: no protection     */

    static const uint8_t xrates[] = { 0x32, 0x04, 0x30, 0x48, 0x60, 0x6c };
    memcpy(p, xrates, sizeof(xrates)); p += sizeof(xrates);  /* Ext rates 24-54 */

    if (ht) {
        /* HT Capabilities (26 B): 20 MHz only, SM-PS disabled, RX MCS 0-7 */
        *p++ = 0x2d; *p++ = 26;
        *p++ = 0x0c; *p++ = 0x00;             /* HT cap info                  */
        *p++ = 0x00;                          /* A-MPDU params                */
        *p++ = 0xff; memset(p, 0, 15); p += 15;   /* MCS set: MCS0-7, 1 SS    */
        memset(p, 0, 2 + 4 + 1); p += 7;      /* ext caps, TxBF, ASEL         */

        /* HT Operation (22 B): primary = ch, 20 MHz, no secondary */
        *p++ = 0x3d; *p++ = 22;
        *p++ = (uint8_t)ch;
        memset(p, 0, 21); p += 21;
    }
    if (he) {
        /* HE Capabilities (ext IE 35): MAC caps 6 B, PHY caps 11 B (20 MHz in
         * 2.4 GHz, no PPE thresholds), MCS/NSS <=80 MHz: 1 SS, HE-MCS 0-9 */
        *p++ = 0xff; *p++ = 1 + 6 + 11 + 4; *p++ = 35;
        memset(p, 0, 6);  p += 6;
        memset(p, 0, 11); p += 11;
        *p++ = 0xfd; *p++ = 0xff;             /* RX HE-MCS map: NSS1 = 0-9    */
        *p++ = 0xfd; *p++ = 0xff;             /* TX HE-MCS map: NSS1 = 0-9    */

        /* HE Operation (ext IE 36): params 3 B, BSS colour 1 B, basic MCS 2 B.
         * BSS colour (1..63) derived from the MAC so co-located boards differ. */
        *p++ = 0xff; *p++ = 1 + 3 + 1 + 2; *p++ = 36;
        *p++ = 0x00; *p++ = 0x00; *p++ = 0x00;
        *p++ = (uint8_t)((s_mac[5] % 63) + 1);
        *p++ = 0xfc; *p++ = 0xff;             /* basic HE-MCS: NSS1 = 0-7     */
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
        /* restore the saved colour tag (dark if never set) */
        led_set(g_led_rgb[0], g_led_rgb[1], g_led_rgb[2]);
    } else {
        s_led = NULL;
        ESP_LOGW(TAG, "no WS2812 on GPIO%d", LED_GPIO);
    }
}

/* Select PHY mode + rate for injected frames. esp_wifi_config_80211_tx() is
 * the only API that can pick the HE20 (11ax) PHY.
 *
 * Note: do NOT esp_wifi_stop() around it. The library dereferences the
 * interface's internal state, which stop() tears down, so the call then
 * returns ESP_FAIL for every rate (verified on hardware). First try it live;
 * if the library refuses that, redo the documented init order — mode set,
 * rate configured, then start — which re-creates the interface state. */
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

/* The transmit loop: duty-cycled injection in 100 ms windows. */
static void tx_task(void *arg)
{
    const int WINDOW_MS = 100;
    int applied_ch  = -1;
    int applied_pwr = -1;
    int applied_rate = -1;

    while (1) {
        if (!g_tx_on) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        int ch  = g_channel;
        int pwr = g_txpwr_dbm;
        int rate = g_rate_idx;

        if (rate != applied_rate) {
            esp_err_t e = apply_rate(rate);
            g_rate_err = e;
            applied_rate = rate;
            applied_ch = -1;
            applied_pwr = -1;
            if (e == ESP_OK) {
                ESP_LOGI(TAG, "rate -> %s (802.11%s)", RATES[rate].name, RATES[rate].std);
            } else {
                ESP_LOGE(TAG, "rate %s rejected: %s", RATES[rate].name, esp_err_to_name(e));
            }
        }
        if (ch != applied_ch) {
            ESP_ERROR_CHECK_WITHOUT_ABORT(
                esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE));
            build_beacon(ch, applied_rate);
            applied_ch = ch;
            ESP_LOGI(TAG, "channel -> %d", ch);
        }
        if (pwr != applied_pwr) {
            /* API takes 0.25 dBm units. */
            esp_wifi_set_max_tx_power((int8_t)(pwr * 4));
            applied_pwr = pwr;
            ESP_LOGI(TAG, "txpower -> %d dBm", pwr);
        }

        int duty   = g_duty;
        if (duty < 0)   duty = 0;
        if (duty > 100) duty = 100;
        int on_ms  = WINDOW_MS * duty / 100;
        int off_ms = WINDOW_MS - on_ms;

        int64_t t0 = esp_timer_get_time();
        while ((esp_timer_get_time() - t0) < (int64_t)on_ms * 1000) {
            /* Live TSF (us since boot, little-endian) so scanners see a sane,
             * monotonically increasing timestamp like a real AP's. */
            uint64_t tsf = (uint64_t)esp_timer_get_time();
            for (int i = 0; i < 8; i++) s_frame[TSF_OFFSET + i] = (uint8_t)(tsf >> (8 * i));
            esp_err_t e = esp_wifi_80211_tx(WIFI_IF_STA, s_frame, s_frame_len, true);
            if (e == ESP_OK) {
                g_frames_ok++;
            } else if (e == ESP_ERR_NO_MEM) {
                /* TX buffers full: normal backpressure at high duty, not a
                 * fault. Yield 1 ms so the radio drains and — since the C6 is
                 * single-core — the console REPL stays responsive. */
                g_tx_bp++;
                vTaskDelay(1);
            } else {
                g_frames_err++;
            }
            /* React quickly to channel / tx changes made from the console. */
            if (!g_tx_on || g_channel != applied_ch || g_rate_idx != applied_rate) break;
        }

        /* One yield per window keeps the idle task (and the WDT) alive; at
         * FREERTOS_HZ=1000 the floor is ~1 ms, so duty=100 stays ~99% on-air. */
        if (off_ms > 0) vTaskDelay(pdMS_TO_TICKS(off_ms));
        else            vTaskDelay(1);
    }
}

/* ---------------------------------------------------------------- console --- */

static int cmd_ch(int argc, char **argv)
{
    if (argc < 2) { printf("usage: ch <1-14>\n"); return 1; }
    int v = atoi(argv[1]);
    if (v < 1 || v > 14) { printf("ERR channel out of range 1-14\n"); return 1; }
    g_channel = v;
    cfg_save();
    printf("OK ch=%d\n", v);
    return 0;
}

static int cmd_duty(int argc, char **argv)
{
    if (argc < 2) { printf("usage: duty <0-100>\n"); return 1; }
    int v = atoi(argv[1]);
    if (v < 0 || v > 100) { printf("ERR duty out of range 0-100\n"); return 1; }
    g_duty = v;
    cfg_save();
    ble_cfg_changed();
    printf("OK duty=%d\n", v);
    return 0;
}

static int cmd_pwr(int argc, char **argv)
{
    if (argc < 2) { printf("usage: pwr <2-20>\n"); return 1; }
    int v = atoi(argv[1]);
    if (v < 2 || v > 20) { printf("ERR pwr out of range 2-20 dBm\n"); return 1; }
    g_txpwr_dbm = v;
    cfg_save();
    printf("OK pwr=%d\n", v);
    return 0;
}

static int cmd_rate(int argc, char **argv)
{
    if (argc < 2) {
        printf("usage: rate <name>\n");
        for (size_t i = 0; i < N_RATES; i++) {
            printf("  %-8s 802.11%s\n", RATES[i].name, RATES[i].std);
        }
        return 1;
    }
    for (size_t i = 0; i < N_RATES; i++) {
        if (strcmp(argv[1], RATES[i].name) == 0) {
            g_rate_idx = (int)i;
            cfg_save();
            printf("OK rate=%s phy=%s\n", RATES[i].name, RATES[i].std);
            return 0;
        }
    }
    printf("ERR unknown rate '%s'\n", argv[1]);
    return 1;
}

static int cmd_tx(int argc, char **argv)
{
    if (argc < 2) { printf("usage: tx <on|off>\n"); return 1; }
    if      (strcmp(argv[1], "on")  == 0) g_tx_on = true;
    else if (strcmp(argv[1], "off") == 0) g_tx_on = false;
    else { printf("ERR expected on|off\n"); return 1; }
    cfg_save();
    ble_cfg_changed();
    printf("OK tx=%s\n", g_tx_on ? "on" : "off");
    return 0;
}

/* Switch radio personality. WiFi and BLE share the one 2.4 GHz radio, so
 * rather than time-slice them (coexistence would chop both waveforms) the
 * choice is saved and the chip restarts with only that stack brought up. */
static int cmd_mode(int argc, char **argv)
{
    int m;
    if (argc < 2) { printf("usage: mode <wifi|ble>\n"); return 1; }
    if      (strcmp(argv[1], "wifi") == 0) m = MODE_WIFI;
    else if (strcmp(argv[1], "ble")  == 0) m = MODE_BLE;
    else { printf("ERR expected wifi|ble\n"); return 1; }
    if (m == g_mode) { printf("OK mode=%s\n", argv[1]); return 0; }
    g_mode = m;
    cfg_save();
    printf("OK mode=%s rebooting\n", argv[1]);
    fflush(stdout);
    vTaskDelay(pdMS_TO_TICKS(150));   /* let the reply drain out of the UART */
    esp_restart();
    return 0;
}

static int cmd_status(int argc, char **argv)
{
    /* One write for the whole line, so a log line from another task can never
     * land in the middle of it and break the host-side parser. */
    static char ble[320];
    ble_status_snprint(ble, sizeof(ble));
    printf("STATUS mac=%02x:%02x:%02x:%02x:%02x:%02x chip=" CONFIG_IDF_TARGET " mode=%s ch=%d duty=%d pwr=%ddBm rate=%s phy=%s rate_err=%s tx=%s "
           "led=%s frames_ok=%llu frames_err=%llu bp=%llu ssid=BEESDR-TX-ch%02d%s\n",
           s_mac[0], s_mac[1], s_mac[2], s_mac[3], s_mac[4], s_mac[5],
           g_mode == MODE_BLE ? "ble" : "wifi", g_channel, g_duty, g_txpwr_dbm, RATES[g_rate_idx].name, RATES[g_rate_idx].std,
           g_rate_err == ESP_OK ? "0" : esp_err_to_name(g_rate_err), g_tx_on ? "on" : "off",
           g_led_name,
           (unsigned long long)g_frames_ok, (unsigned long long)g_frames_err,
           (unsigned long long)g_tx_bp, g_channel, ble);
    return 0;
}

static int cmd_led(int argc, char **argv)
{
    if (argc < 2) {
        printf("usage: led <off|red|green|blue|yellow|cyan|magenta|white|orange>"
               " | led #rrggbb | led rgb <r> <g> <b>\n");
        return 1;
    }
    int r = 0, g = 0, b = 0;
    const int L = LED_LVL;
    if (argv[1][0] == '#') {
        /* Display colour as picked in a UI. Scaled to LED_LVL so it lands at the
         * same brightness/current as the named colours; rounded up so any
         * non-zero channel stays lit. Reported back verbatim as led=#rrggbb. */
        unsigned v;
        char *end;
        if (strlen(argv[1]) != 7) { printf("ERR bad hex colour '%s'\n", argv[1]); return 1; }
        v = (unsigned)strtoul(argv[1] + 1, &end, 16);
        if (*end != '\0') { printf("ERR bad hex colour '%s'\n", argv[1]); return 1; }
        r = (int)(((v >> 16) & 0xff) * L + 254) / 255;
        g = (int)(((v >>  8) & 0xff) * L + 254) / 255;
        b = (int)(( v        & 0xff) * L + 254) / 255;
        snprintf(g_led_name, sizeof(g_led_name), "#%06x", v);
    } else if (strcmp(argv[1], "rgb") == 0 && argc >= 5) {
        r = atoi(argv[2]); g = atoi(argv[3]); b = atoi(argv[4]);
        snprintf(g_led_name, sizeof(g_led_name), "rgb");
    } else {
        const struct { const char *n; int r, g, b; } c[] = {
            {"off",0,0,0}, {"red",L,0,0}, {"green",0,L,0}, {"blue",0,0,L},
            {"yellow",L,L,0}, {"cyan",0,L,L}, {"magenta",L,0,L},
            {"white",L,L,L}, {"orange",L,L/4,0},
        };
        int found = 0;
        for (size_t i = 0; i < sizeof(c)/sizeof(c[0]); i++) {
            if (strcmp(argv[1], c[i].n) == 0) {
                r=c[i].r; g=c[i].g; b=c[i].b; found=1;
                snprintf(g_led_name, sizeof(g_led_name), "%s", c[i].n);
                break;
            }
        }
        if (!found) { printf("ERR unknown color '%s'\n", argv[1]); return 1; }
    }
    bool changed = g_led_rgb[0] != r || g_led_rgb[1] != g || g_led_rgb[2] != b;
    g_led_rgb[0] = (uint8_t)r; g_led_rgb[1] = (uint8_t)g; g_led_rgb[2] = (uint8_t)b;
    led_set(g_led_rgb[0], g_led_rgb[1], g_led_rgb[2]);
    /* Only write flash on an actual change, to keep wear negligible. */
    if (changed) cfg_save();
    printf("OK led=%s\n", g_led_name);
    return 0;
}

static void register_commands(void)
{
    const esp_console_cmd_t cmds[] = {
        {.command = "mode",   .help = "radio: wifi | ble (reboots)",   .func = cmd_mode},
        {.command = "ch",     .help = "set WiFi channel 1-14",         .func = cmd_ch},
        {.command = "duty",   .help = "on-air duty cycle 0-100 pct",   .func = cmd_duty},
        {.command = "pwr",    .help = "max TX power 2-20 dBm",          .func = cmd_pwr},
        {.command = "rate",   .help = "PHY rate / standard b,g,n,ax",   .func = cmd_rate},
        {.command = "tx",     .help = "enable/pause injection on/off", .func = cmd_tx},
        {.command = "status", .help = "print current TX state",         .func = cmd_status},
        {.command = "led",    .help = "set RGB LED colour",             .func = cmd_led},
    };
    for (size_t i = 0; i < sizeof(cmds)/sizeof(cmds[0]); i++) {
        ESP_ERROR_CHECK(esp_console_cmd_register(&cmds[i]));
    }
    ble_register_commands();
}

/* ------------------------------------------------------------------- main --- */

/* Bring up the WiFi stack and the injection task (WiFi mode only). */
static void wifi_tx_start(void)
{
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    /* Manual country policy with the full 1-14 span so injection is not
     * silently clamped on the upper channels. Channels 12-14 are region-
     * restricted for normal use; this is deliberate lab injection. */
    wifi_country_t country = {
        .cc = "01", .schan = 1, .nchan = 14, .policy = WIFI_COUNTRY_POLICY_MANUAL,
    };
    ESP_ERROR_CHECK(esp_wifi_set_country(&country));

    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    /* Enable every 2.4 GHz standard the chip has, so any rate in RATES[] —
     * including HE20 (11ax) where the radio supports it — is permitted. */
    ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA,
        WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N
#if SOC_WIFI_HE_SUPPORT
        | WIFI_PROTOCOL_11AX
#endif
        ));
    ESP_ERROR_CHECK(esp_wifi_start());

    build_beacon(g_channel, g_rate_idx);
    ESP_LOGI(TAG, "BEESDR " CONFIG_IDF_TARGET " WiFi TX up. mac=%02x:%02x:%02x:%02x:%02x:%02x "
             "default ch=%d", s_mac[0], s_mac[1], s_mac[2], s_mac[3],
             s_mac[4], s_mac[5], g_channel);

    xTaskCreatePinnedToCore(tx_task, "tx_task", 4096, NULL, 5, NULL, 0);
}

void app_main(void)
{
    esp_err_t nv = nvs_flash_init();
    if (nv == ESP_ERR_NVS_NO_FREE_PAGES || nv == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nv = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nv);
    cfg_load();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    ESP_ERROR_CHECK(esp_read_mac(s_mac, ESP_MAC_WIFI_STA));
    led_init();

    if (g_mode == MODE_BLE) {
        ble_tx_start();
        ESP_LOGI(TAG, "BEESDR " CONFIG_IDF_TARGET " BLE TX up. mac=%02x:%02x:%02x:%02x:%02x:%02x",
                 s_mac[0], s_mac[1], s_mac[2], s_mac[3], s_mac[4], s_mac[5]);
    } else {
        wifi_tx_start();
    }
    /* Interactive REPL on the USB-Serial-JTAG console. */
    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_cfg = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_cfg.prompt = "beesdr-tx>";
#if defined(CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG)
    esp_console_dev_usb_serial_jtag_config_t hw =
        ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_serial_jtag(&hw, &repl_cfg, &repl));
#else
    esp_console_dev_uart_config_t hw = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&hw, &repl_cfg, &repl));
#endif
    register_commands();
    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}
