/*
 * beacon_tx.c — BEESDR ESP32-C6 WiFi transmitter for SDR/AoA lab work.
 *
 * Puts the C6 into a raw-injection "transmitter mode": it sits on one 2.4 GHz
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
 *     rate  <name>     PHY rate: 1m 2m 5m 11m 6m 9m 12m 18m 24m 36m 48m 54m
 *                      mcs0..mcs7  (6m default — legacy 20 MHz OFDM)
 *     tx    <on|off>   enable / pause injection
 *     status           print current state + frame/error counters
 *
 * One firmware image serves every board; boards are told apart by channel
 * (assigned per-board with tools/esp32c6_ctl.py) and by a MAC-derived BSSID.
 *
 * BEESDR drone-RF R&D — Experiments/ESP32C6.
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
#include "esp_console.h"
#include "esp_rom_sys.h"      /* esp_rom_delay_us */
#include "led_strip.h"        /* addressable RGB status LED (WS2812) */

/* ---- Compile-time default channel (override at runtime with `ch`). The flash
 * helper also assigns a per-board channel over serial right after flashing, so
 * this is only the pre-assignment boot value. ---- */
#ifndef DEFAULT_CHANNEL
#define DEFAULT_CHANNEL 6
#endif

/* Addressable RGB LED (WS2812) pin — GPIO8 on the ESP32-C6-WROOM-1 /
 * DevKitC-1 layout. Used as a per-board colour tag so boards are told
 * apart at a glance; the dashboard sets it from a MAC->colour map. */
#ifndef LED_GPIO
#define LED_GPIO 8
#endif
#define LED_LVL 48   /* per-channel brightness 0-255; low to limit current */

static const char *TAG = "beesdr-tx";

/* ---- Shared TX config. Plain scalars written from the console task and read
 * by the TX task; word-sized and single-writer per field, so no lock needed for
 * this lab tool. ---- */
static volatile int      g_channel   = DEFAULT_CHANNEL;  /* 1..14              */
static volatile int      g_duty      = 100;              /* 0..100 percent     */
static volatile int      g_txpwr_dbm = 20;               /* 2..20 dBm          */
static volatile bool     g_tx_on     = true;
static volatile wifi_phy_rate_t g_rate = WIFI_PHY_RATE_6M;
static const char * volatile g_rate_name = "6m";

/* ---- Counters for `status`. ---- */
static volatile uint64_t g_frames_ok  = 0;
static volatile uint64_t g_frames_err = 0;
static volatile uint64_t g_tx_bp      = 0;   /* TX-buffer backpressure (normal) */

static uint8_t s_mac[6];
static uint8_t s_frame[128];
static size_t  s_frame_len;

static led_strip_handle_t s_led = NULL;
static char g_led_name[12] = "off";

/* Build a valid beacon frame carrying the current channel in its SSID and
 * DS-parameter IE, so a sniffer (or the SDR operator) can read the channel off
 * the air and tell boards apart. */
static void build_beacon(int ch)
{
    uint8_t *p = s_frame;

    *p++ = 0x80; *p++ = 0x00;                 /* FC: mgmt / beacon            */
    *p++ = 0x00; *p++ = 0x00;                 /* duration                     */
    memset(p, 0xff, 6); p += 6;               /* addr1 DA = broadcast         */
    memcpy(p, s_mac, 6); p += 6;              /* addr2 SA                     */
    memcpy(p, s_mac, 6); p += 6;              /* addr3 BSSID                  */
    *p++ = 0x00; *p++ = 0x00;                 /* seq (overwritten by HW)      */

    memset(p, 0, 8); p += 8;                  /* timestamp                    */
    *p++ = 0x64; *p++ = 0x00;                 /* beacon interval = 100 TU     */
    *p++ = 0x01; *p++ = 0x00;                 /* capability = ESS             */

    char ssid[24];
    int sl = snprintf(ssid, sizeof(ssid), "BEESDR-TX-ch%02d", ch);
    *p++ = 0x00; *p++ = (uint8_t)sl;          /* SSID IE                      */
    memcpy(p, ssid, sl); p += sl;

    static const uint8_t rates[] = {
        0x01, 0x08, 0x82, 0x84, 0x8b, 0x96, 0x0c, 0x12, 0x18, 0x24
    };                                        /* supported rates IE (8 rates) */
    memcpy(p, rates, sizeof(rates)); p += sizeof(rates);

    *p++ = 0x03; *p++ = 0x01; *p++ = (uint8_t)ch;  /* DS Parameter set = ch   */

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
        led_set(0, 0, 0);   /* start dark; dashboard sets the board colour */
    } else {
        s_led = NULL;
        ESP_LOGW(TAG, "no WS2812 on GPIO%d", LED_GPIO);
    }
}

/* The transmit loop: duty-cycled injection in 100 ms windows. */
static void tx_task(void *arg)
{
    const int WINDOW_MS = 100;
    int applied_ch  = -1;
    int applied_pwr = -1;
    wifi_phy_rate_t applied_rate = (wifi_phy_rate_t)-1;

    while (1) {
        if (!g_tx_on) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        int ch  = g_channel;
        int pwr = g_txpwr_dbm;
        wifi_phy_rate_t rate = g_rate;

        if (ch != applied_ch) {
            ESP_ERROR_CHECK_WITHOUT_ABORT(
                esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE));
            build_beacon(ch);
            applied_ch = ch;
            ESP_LOGI(TAG, "channel -> %d", ch);
        }
        if (pwr != applied_pwr) {
            /* API takes 0.25 dBm units. */
            esp_wifi_set_max_tx_power((int8_t)(pwr * 4));
            applied_pwr = pwr;
            ESP_LOGI(TAG, "txpower -> %d dBm", pwr);
        }
        if (rate != applied_rate) {
            esp_wifi_config_80211_tx_rate(WIFI_IF_STA, rate);
            applied_rate = rate;
        }

        int duty   = g_duty;
        if (duty < 0)   duty = 0;
        if (duty > 100) duty = 100;
        int on_ms  = WINDOW_MS * duty / 100;
        int off_ms = WINDOW_MS - on_ms;

        int64_t t0 = esp_timer_get_time();
        while ((esp_timer_get_time() - t0) < (int64_t)on_ms * 1000) {
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
            if (!g_tx_on || g_channel != applied_ch) break;
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
    printf("OK ch=%d\n", v);
    return 0;
}

static int cmd_duty(int argc, char **argv)
{
    if (argc < 2) { printf("usage: duty <0-100>\n"); return 1; }
    int v = atoi(argv[1]);
    if (v < 0 || v > 100) { printf("ERR duty out of range 0-100\n"); return 1; }
    g_duty = v;
    printf("OK duty=%d\n", v);
    return 0;
}

static int cmd_pwr(int argc, char **argv)
{
    if (argc < 2) { printf("usage: pwr <2-20>\n"); return 1; }
    int v = atoi(argv[1]);
    if (v < 2 || v > 20) { printf("ERR pwr out of range 2-20 dBm\n"); return 1; }
    g_txpwr_dbm = v;
    printf("OK pwr=%d\n", v);
    return 0;
}

static int cmd_rate(int argc, char **argv)
{
    if (argc < 2) { printf("usage: rate <1m|2m|5m|11m|6m|9m|12m|18m|24m|36m|48m|54m|mcs0..mcs7>\n"); return 1; }
    const char *r = argv[1];
    struct { const char *n; wifi_phy_rate_t v; } map[] = {
        {"1m",  WIFI_PHY_RATE_1M_L}, {"2m",  WIFI_PHY_RATE_2M_L},
        {"5m",  WIFI_PHY_RATE_5M_L}, {"11m", WIFI_PHY_RATE_11M_L},
        {"6m",  WIFI_PHY_RATE_6M},   {"9m",  WIFI_PHY_RATE_9M},
        {"12m", WIFI_PHY_RATE_12M},  {"18m", WIFI_PHY_RATE_18M},
        {"24m", WIFI_PHY_RATE_24M},  {"36m", WIFI_PHY_RATE_36M},
        {"48m", WIFI_PHY_RATE_48M},  {"54m", WIFI_PHY_RATE_54M},
        {"mcs0", WIFI_PHY_RATE_MCS0_LGI}, {"mcs1", WIFI_PHY_RATE_MCS1_LGI},
        {"mcs2", WIFI_PHY_RATE_MCS2_LGI}, {"mcs3", WIFI_PHY_RATE_MCS3_LGI},
        {"mcs4", WIFI_PHY_RATE_MCS4_LGI}, {"mcs5", WIFI_PHY_RATE_MCS5_LGI},
        {"mcs6", WIFI_PHY_RATE_MCS6_LGI}, {"mcs7", WIFI_PHY_RATE_MCS7_LGI},
    };
    for (size_t i = 0; i < sizeof(map)/sizeof(map[0]); i++) {
        if (strcmp(r, map[i].n) == 0) {
            g_rate = map[i].v;
            g_rate_name = map[i].n;
            printf("OK rate=%s\n", r);
            return 0;
        }
    }
    printf("ERR unknown rate '%s'\n", r);
    return 1;
}

static int cmd_tx(int argc, char **argv)
{
    if (argc < 2) { printf("usage: tx <on|off>\n"); return 1; }
    if      (strcmp(argv[1], "on")  == 0) g_tx_on = true;
    else if (strcmp(argv[1], "off") == 0) g_tx_on = false;
    else { printf("ERR expected on|off\n"); return 1; }
    printf("OK tx=%s\n", g_tx_on ? "on" : "off");
    return 0;
}

static int cmd_status(int argc, char **argv)
{
    printf("STATUS mac=%02x:%02x:%02x:%02x:%02x:%02x ch=%d duty=%d pwr=%ddBm rate=%s tx=%s "
           "led=%s frames_ok=%llu frames_err=%llu bp=%llu ssid=BEESDR-TX-ch%02d\n",
           s_mac[0], s_mac[1], s_mac[2], s_mac[3], s_mac[4], s_mac[5],
           g_channel, g_duty, g_txpwr_dbm, g_rate_name, g_tx_on ? "on" : "off",
           g_led_name,
           (unsigned long long)g_frames_ok, (unsigned long long)g_frames_err,
           (unsigned long long)g_tx_bp, g_channel);
    return 0;
}

static int cmd_led(int argc, char **argv)
{
    if (argc < 2) {
        printf("usage: led <off|red|green|blue|yellow|cyan|magenta|white|orange>"
               " | led rgb <r> <g> <b>\n");
        return 1;
    }
    int r = 0, g = 0, b = 0;
    const int L = LED_LVL;
    if (strcmp(argv[1], "rgb") == 0 && argc >= 5) {
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
    led_set((uint8_t)r, (uint8_t)g, (uint8_t)b);
    printf("OK led=%s\n", g_led_name);
    return 0;
}

static void register_commands(void)
{
    const esp_console_cmd_t cmds[] = {
        {.command = "ch",     .help = "set WiFi channel 1-14",         .func = cmd_ch},
        {.command = "duty",   .help = "on-air duty cycle 0-100 pct",   .func = cmd_duty},
        {.command = "pwr",    .help = "max TX power 2-20 dBm",          .func = cmd_pwr},
        {.command = "rate",   .help = "PHY rate (6m default OFDM)",     .func = cmd_rate},
        {.command = "tx",     .help = "enable/pause injection on/off", .func = cmd_tx},
        {.command = "status", .help = "print current TX state",         .func = cmd_status},
        {.command = "led",    .help = "set RGB LED colour",             .func = cmd_led},
    };
    for (size_t i = 0; i < sizeof(cmds)/sizeof(cmds[0]); i++) {
        ESP_ERROR_CHECK(esp_console_cmd_register(&cmds[i]));
    }
}

/* ------------------------------------------------------------------- main --- */

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

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
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_ERROR_CHECK(esp_read_mac(s_mac, ESP_MAC_WIFI_STA));
    build_beacon(g_channel);
    led_init();

    ESP_LOGI(TAG, "BEESDR ESP32-C6 WiFi TX up. mac=%02x:%02x:%02x:%02x:%02x:%02x "
             "default ch=%d", s_mac[0], s_mac[1], s_mac[2], s_mac[3],
             s_mac[4], s_mac[5], g_channel);

    xTaskCreatePinnedToCore(tx_task, "tx_task", 4096, NULL, 5, NULL, 0);

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
