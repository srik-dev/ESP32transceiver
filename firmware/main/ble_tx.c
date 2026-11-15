/*
 * ble_tx.c — Bluetooth LE transmit mode for the BEESDR ESP32-Tx firmware.
 *
 * The chip has one 2.4 GHz radio, so a board is either a WiFi transmitter or a
 * BLE one: `mode wifi|ble` (beacon_tx.c) stores the choice and reboots into it,
 * and only that stack is brought up. Everything here is driven from the same
 * serial console and persisted in the same NVS namespace.
 *
 * Three kinds of BLE transmission, selected with `ble <adv|ext|dtm>`:
 *
 *   adv  Bluetooth 4 legacy advertising. ADV_NONCONN_IND on the primary
 *        advertising channels 37/38/39 (2402/2426/2480 MHz), always LE 1M.
 *        Any phone or BLE scanner lists the board by name.
 *   ext  Bluetooth 5 extended advertising. ADV_EXT_IND on 37/38/39 pointing at
 *        an AUX_ADV_IND on a data channel the controller picks. `phy` chooses:
 *          1m  primary 1M    / secondary 1M
 *          2m  primary 1M    / secondary 2M   (2M is not allowed on primary)
 *          s8  primary Coded / secondary Coded, S=8 (125 kb/s, long range)
 *          s2  primary Coded / secondary Coded, S=2 (500 kb/s)
 *        `len` pads the advertising data (manufacturer AD) to lengthen the burst.
 *   dtm  Direct Test Mode (HCI LE Transmitter Test v2). Back-to-back test
 *        packets on ONE fixed RF channel 0-39 (2402 + 2k MHz), any PHY (1M, 2M,
 *        Coded S=8, Coded S=2), with a chosen payload pattern and length. Not
 *        advertising — invisible to phones — but a steady, known, high-duty
 *        emitter, which is what SDR / AoA bench work usually wants. `duty`
 *        gates it in 100 ms windows exactly like the WiFi injector.
 *
 * Console commands (all saved to NVS, all accepted in either mode):
 *     ble   <adv|ext|dtm>          transmission kind
 *     phy   <1m|2m|s8|s2>          PHY (ext, dtm; adv is always 1m)
 *     bch   <0-39>                 DTM RF channel index, freq = 2402 + 2*n MHz
 *     advch <all|37,38,39 subset>  advertising channels used (adv, ext)
 *     itvl  <20-10240>             advertising interval in ms (adv, ext)
 *     bpwr  <dBm>                  BLE TX power, snapped down to a 3 dB step
 *     pat   <name>                 DTM payload pattern
 *     len   <0-255>                DTM payload bytes / ext advertising-data bytes
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_console.h"
#include "esp_bt.h"
#include "sdkconfig.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/util/util.h"

#include "ble_tx.h"

static const char *TAG = "beesdr-ble";

enum { KIND_ADV, KIND_EXT, KIND_DTM, N_KINDS };
static const char *const KIND_NAMES[N_KINDS] = {"adv", "ext", "dtm"};

enum { PHY_1M, PHY_2M, PHY_S8, PHY_S2, N_PHYS };
static const char *const PHY_NAMES[N_PHYS] = {"1m", "2m", "s8", "s2"};

/* DTM payload patterns, indexed by the HCI LE Transmitter Test payload code. */
static const char *const PAT_NAMES[] = {
    "prbs9", "11110000", "10101010", "prbs15", "ones", "zeros", "00001111", "01010101",
};
#define N_PATS (sizeof(PAT_NAMES) / sizeof(PAT_NAMES[0]))

/* esp_power_level_t index -> dBm. Same indices on every target; the C6
 * controller's range starts at -15 dBm (index 3), the S3's at -24 dBm. */
static const int8_t PWR_DBM[] = {-24, -21, -18, -15, -12, -9, -6, -3, 0, 3, 6, 9, 12, 15, 18, 20};
#define N_PWR ((int)(sizeof(PWR_DBM) / sizeof(PWR_DBM[0])))
#if CONFIG_IDF_TARGET_ESP32S3
#define PWR_MIN_IDX 0
#else
#define PWR_MIN_IDX 3
#endif

#define ITVL_MIN_MS 20
#define ITVL_MAX_MS 10240
#define EXT_DATA_MAX 251     /* keeps the data in a single AUX_ADV_IND */

/* ---- Settings (console task writes, BLE task reads; word-sized scalars). ---- */
static volatile int g_kind   = KIND_ADV;
static volatile int g_phy    = PHY_1M;
static volatile int g_bch    = 19;     /* DTM RF channel index: 2440 MHz     */
static volatile int g_advch  = 0x07;   /* bit0=37 bit1=38 bit2=39            */
static volatile int g_itvl   = 20;     /* ms                                 */
static volatile int g_bpwr   = 9;      /* dBm, always one of PWR_DBM[]       */
static volatile int g_pat    = 0;
static volatile int g_len    = 37;

/* ---- Runtime state. ---- */
static volatile uint32_t s_gen = 1;          /* bumped on every config change */
static volatile bool     s_synced = false;
static volatile bool     s_running = false;  /* ble_tx_start() was called     */
static volatile const char *s_state = "off";
static volatile const char *s_coding = "-";
static char              s_err[32] = "0";
static TaskHandle_t      s_task = NULL;
static char              s_name[20];
static bool              s_adv_cfgd = false;
static bool              s_dtm_on = false;

static int pwr_idx(int dbm)
{
    int idx = PWR_MIN_IDX;
    for (int i = PWR_MIN_IDX; i < N_PWR; i++) {
        if (PWR_DBM[i] <= dbm) idx = i;
    }
    return idx;
}

static void set_err(const char *stage, int rc)
{
    if (rc == 0) strcpy(s_err, "0");
    else         snprintf(s_err, sizeof(s_err), "%s:%d", stage, rc);
    if (rc != 0) ESP_LOGE(TAG, "%s failed rc=%d", stage, rc);
}

static void advch_str(int mask, char *out, size_t n)
{
    out[0] = '\0';
    for (int i = 0; i < 3; i++) {
        if (mask & (1 << i)) {
            size_t l = strlen(out);
            snprintf(out + l, n - l, "%s%d", l ? "," : "", 37 + i);
        }
    }
}

/* ------------------------------------------------------------ persistence --- */

void ble_cfg_save(nvs_handle_t h)
{
    nvs_set_u8 (h, "bkind", (uint8_t)g_kind);
    nvs_set_u8 (h, "bphy",  (uint8_t)g_phy);
    nvs_set_u8 (h, "bch",   (uint8_t)g_bch);
    nvs_set_u8 (h, "advch", (uint8_t)g_advch);
    nvs_set_u16(h, "itvl",  (uint16_t)g_itvl);
    nvs_set_i8 (h, "bpwr",  (int8_t)g_bpwr);
    nvs_set_u8 (h, "pat",   (uint8_t)g_pat);
    nvs_set_u8 (h, "blen",  (uint8_t)g_len);
}

void ble_cfg_load(nvs_handle_t h)
{
    uint8_t v;
    uint16_t w;
    int8_t s;
    if (nvs_get_u8(h, "bkind", &v) == ESP_OK && v < N_KINDS)          g_kind  = v;
    if (nvs_get_u8(h, "bphy",  &v) == ESP_OK && v < N_PHYS)           g_phy   = v;
    if (nvs_get_u8(h, "bch",   &v) == ESP_OK && v <= 39)              g_bch   = v;
    if (nvs_get_u8(h, "advch", &v) == ESP_OK && v >= 1 && v <= 7)     g_advch = v;
    if (nvs_get_u16(h, "itvl", &w) == ESP_OK && w >= ITVL_MIN_MS && w <= ITVL_MAX_MS) g_itvl = w;
    if (nvs_get_i8(h, "bpwr",  &s) == ESP_OK)                         g_bpwr  = PWR_DBM[pwr_idx(s)];
    if (nvs_get_u8(h, "pat",   &v) == ESP_OK && v < N_PATS)           g_pat   = v;
    if (nvs_get_u8(h, "blen",  &v) == ESP_OK)                         g_len   = v;
}

/* ---------------------------------------------------------------- on air --- */

static void ble_stop(void)
{
    if (s_dtm_on) {
        ble_gap_dtm_stop();
        s_dtm_on = false;
    }
    if (s_adv_cfgd) {
        ble_gap_ext_adv_stop(0);
        ble_gap_ext_adv_remove(0);
        s_adv_cfgd = false;
    }
}

static void apply_power(int dbm)
{
    esp_power_level_t lvl = (esp_power_level_t)pwr_idx(dbm);
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, lvl);   /* DTM uses this one */
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, lvl);
}

/* Advertising data: Flags, Complete Local Name, and — extended only — a
 * manufacturer-specific AD (test company ID 0xFFFF) padding the total to `len`. */
static int build_adv_data(uint8_t *buf, int kind, int len)
{
    int n = 0;
    int nl = (int)strlen(s_name);
    buf[n++] = 2; buf[n++] = 0x01; buf[n++] = 0x06;
    buf[n++] = (uint8_t)(nl + 1); buf[n++] = 0x09;
    memcpy(buf + n, s_name, nl); n += nl;
    if (len > EXT_DATA_MAX) len = EXT_DATA_MAX;
    if (kind == KIND_EXT && len >= n + 4) {
        int pad = len - n - 4;
        buf[n++] = (uint8_t)(pad + 3); buf[n++] = 0xff; buf[n++] = 0xff; buf[n++] = 0xff;
        for (int i = 0; i < pad; i++) buf[n++] = (uint8_t)i;
    }
    return n;
}

static int adv_start(int kind, int phy, int advch, int itvl_ms, int pwr)
{
    struct ble_gap_ext_adv_params p;
    memset(&p, 0, sizeof(p));
    p.legacy_pdu = (kind == KIND_ADV);          /* non-conn, non-scan: ADV_NONCONN_IND */
    p.itvl_min = p.itvl_max = (uint32_t)itvl_ms * 1000 / 625;
    p.channel_map = (uint8_t)advch;
    p.own_addr_type = BLE_OWN_ADDR_PUBLIC;
    p.tx_power = (int8_t)pwr;
    p.sid = 0;
    bool coded = (kind == KIND_EXT) && (phy == PHY_S8 || phy == PHY_S2);
    p.primary_phy   = coded ? BLE_HCI_LE_PHY_CODED : BLE_HCI_LE_PHY_1M;
    p.secondary_phy = coded ? BLE_HCI_LE_PHY_CODED
                    : (kind == KIND_EXT && phy == PHY_2M) ? BLE_HCI_LE_PHY_2M
                    : BLE_HCI_LE_PHY_1M;

    int rc = -1;
    s_coding = "-";
#if MYNEWT_VAL(BLE_EXT_ADV_V2)
    if (coded) {
        /* Set Extended Advertising Parameters v2 (BT 5.4): 3 = S=2 required,
         * 4 = S=8 required. Only this command can pick the coding. */
        p.primary_phy_opt = p.secondary_phy_opt = (phy == PHY_S2) ? 3 : 4;
        rc = ble_gap_ext_adv_configure(0, &p, NULL, NULL, NULL);
        if (rc == 0) {
            s_coding = (phy == PHY_S2) ? "s2" : "s8";
        } else {
            ESP_LOGW(TAG, "controller cannot select coded S=%d (rc=%d), using its default",
                     phy == PHY_S2 ? 2 : 8, rc);
            p.primary_phy_opt = p.secondary_phy_opt = 0;
        }
    }
#endif
    if (rc != 0) {
        rc = ble_gap_ext_adv_configure(0, &p, NULL, NULL, NULL);
        if (rc == 0 && coded) s_coding = "ctrl";    /* controller's choice (S=8 typ.) */
    }
    if (rc != 0) { set_err("adv_cfg", rc); return rc; }
    s_adv_cfgd = true;

    uint8_t data[EXT_DATA_MAX];
    int n = build_adv_data(data, kind, g_len);
    struct os_mbuf *om = os_msys_get_pkthdr(n, 0);
    if (!om) { set_err("adv_mbuf", BLE_HS_ENOMEM); return BLE_HS_ENOMEM; }
    rc = os_mbuf_append(om, data, n);
    if (rc != 0) { os_mbuf_free_chain(om); set_err("adv_mbuf", rc); return rc; }
    rc = ble_gap_ext_adv_set_data(0, om);
    if (rc != 0) { set_err("adv_data", rc); return rc; }

    rc = ble_gap_ext_adv_start(0, 0, 0);
    if (rc != 0) { set_err("adv_start", rc); return rc; }
    return 0;
}

static int dtm_start(int ch, int len, int pat, int phy)
{
    /* LE Transmitter Test v2 PHY: 1 = 1M, 2 = 2M, 3 = Coded S=8, 4 = Coded S=2 */
    static const uint8_t hci_phy[N_PHYS] = {1, 2, 3, 4};
    int rc = ble_gap_dtm_enh_tx_start((uint8_t)ch, (uint8_t)len, (uint8_t)pat, hci_phy[phy]);
    if (rc == 0) s_dtm_on = true;
    set_err("dtm_start", rc);
    return rc;
}

static void wait_ms(int ms)
{
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(ms > 0 ? ms : 1));
}

static void ble_task(void *arg)
{
    const int WINDOW_MS = 100;
    uint32_t gen = 0;
    bool dtm_active = false;
    int kind = 0, phy = 0, bch = 0, pat = 0, len = 0;

    while (!s_synced) vTaskDelay(pdMS_TO_TICKS(20));

    while (1) {
        if (gen != s_gen) {
            gen = s_gen;
            ble_stop();
            dtm_active = false;
            strcpy(s_err, "0");
            s_coding = "-";
            kind = g_kind; phy = g_phy; bch = g_bch; pat = g_pat; len = g_len;
            int pwr = g_bpwr;
            apply_power(pwr);
            if (!g_tx_on) {
                s_state = "idle";
            } else if (kind == KIND_DTM) {
                dtm_active = true;
                s_state = "dtm";
                if (phy == PHY_S8 || phy == PHY_S2) s_coding = PHY_NAMES[phy];
            } else {
                int rc = adv_start(kind, phy, g_advch, g_itvl, pwr);
                s_state = (rc == 0) ? "adv" : "error";
            }
            ESP_LOGD(TAG, "apply: tx=%d kind=%s phy=%s state=%s err=%s", (int)g_tx_on,
                     KIND_NAMES[kind], PHY_NAMES[phy], (const char *)s_state, s_err);
        }

        if (!dtm_active) { wait_ms(200); continue; }

        int duty = g_duty;
        if (duty >= 100) {
            if (!s_dtm_on && dtm_start(bch, len, pat, phy) != 0) { s_state = "error"; dtm_active = false; }
            wait_ms(200);
        } else if (duty <= 0) {
            if (s_dtm_on) { ble_gap_dtm_stop(); s_dtm_on = false; }
            wait_ms(200);
        } else {
            int on_ms = WINDOW_MS * duty / 100;
            if (dtm_start(bch, len, pat, phy) != 0) { s_state = "error"; dtm_active = false; continue; }
            wait_ms(on_ms);
            ble_gap_dtm_stop();
            s_dtm_on = false;
            if (gen == s_gen) wait_ms(WINDOW_MS - on_ms);
        }
    }
}

static void on_sync(void)
{
    int rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) set_err("addr", rc);
    s_synced = true;
}

static void on_reset(int reason)
{
    ESP_LOGE(TAG, "BLE host reset, reason=%d", reason);
}

static void host_task(void *arg)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void ble_tx_start(void)
{
    s_state = "init";
    esp_err_t e = nimble_port_init();
    if (e != ESP_OK) {
        set_err("nimble_init", (int)e);
        s_state = "error";
        return;
    }
    ble_hs_cfg.sync_cb = on_sync;
    ble_hs_cfg.reset_cb = on_reset;
    nimble_port_freertos_init(host_task);
    s_running = true;
    xTaskCreate(ble_task, "ble_task", 4096, NULL, 5, &s_task);
}

void ble_cfg_changed(void)
{
    s_gen++;
    if (s_task) xTaskNotifyGive(s_task);
}

/* ---------------------------------------------------------------- console --- */

static void changed(void)
{
    cfg_save();
    ble_cfg_changed();
}

static int lookup(const char *s, const char *const *names, int n)
{
    for (int i = 0; i < n; i++) {
        if (strcmp(s, names[i]) == 0) return i;
    }
    return -1;
}

static int cmd_ble(int argc, char **argv)
{
    int v = (argc >= 2) ? lookup(argv[1], KIND_NAMES, N_KINDS) : -1;
    if (v < 0) { printf("%s ble <adv|ext|dtm>\n", argc < 2 ? "usage:" : "ERR expected"); return 1; }
    g_kind = v;
    changed();
    printf("OK ble=%s\n", KIND_NAMES[v]);
    return 0;
}

static int cmd_phy(int argc, char **argv)
{
    int v = (argc >= 2) ? lookup(argv[1], PHY_NAMES, N_PHYS) : -1;
    if (v < 0) { printf("%s phy <1m|2m|s8|s2>\n", argc < 2 ? "usage:" : "ERR expected"); return 1; }
    g_phy = v;
    changed();
    printf("OK phy=%s\n", PHY_NAMES[v]);
    return 0;
}

static int cmd_bch(int argc, char **argv)
{
    if (argc < 2) { printf("usage: bch <0-39>  (RF channel, 2402 + 2*n MHz)\n"); return 1; }
    char *end;
    long v = strtol(argv[1], &end, 10);
    if (*end != '\0' || v < 0 || v > 39) { printf("ERR bch out of range 0-39\n"); return 1; }
    g_bch = (int)v;
    changed();
    printf("OK bch=%d freq=%d\n", (int)v, 2402 + 2 * (int)v);
    return 0;
}

static int cmd_advch(int argc, char **argv)
{
    if (argc < 2) { printf("usage: advch <all|37,38,39 subset>\n"); return 1; }
    int mask = 0;
    if (strcmp(argv[1], "all") == 0) {
        mask = 7;
    } else {
        char tmp[16];
        snprintf(tmp, sizeof(tmp), "%s", argv[1]);
        for (char *t = strtok(tmp, ","); t; t = strtok(NULL, ",")) {
            int c = atoi(t);
            if (c < 37 || c > 39) { printf("ERR advch takes 37, 38, 39 or all\n"); return 1; }
            mask |= 1 << (c - 37);
        }
    }
    if (mask == 0) { printf("ERR advch needs at least one channel\n"); return 1; }
    g_advch = mask;
    changed();
    char s[12];
    advch_str(mask, s, sizeof(s));
    printf("OK advch=%s\n", s);
    return 0;
}

static int cmd_itvl(int argc, char **argv)
{
    if (argc < 2) { printf("usage: itvl <%d-%d> ms\n", ITVL_MIN_MS, ITVL_MAX_MS); return 1; }
    int v = atoi(argv[1]);
    if (v < ITVL_MIN_MS || v > ITVL_MAX_MS) {
        printf("ERR itvl out of range %d-%d ms\n", ITVL_MIN_MS, ITVL_MAX_MS);
        return 1;
    }
    g_itvl = v;
    changed();
    printf("OK itvl=%d\n", v);
    return 0;
}

static int cmd_bpwr(int argc, char **argv)
{
    int lo = PWR_DBM[PWR_MIN_IDX], hi = PWR_DBM[N_PWR - 1];
    if (argc < 2) { printf("usage: bpwr <%d..%d> dBm (3 dB steps)\n", lo, hi); return 1; }
    char *end;
    long v = strtol(argv[1], &end, 10);
    if (*end != '\0' || v < lo || v > hi) { printf("ERR bpwr out of range %d..%d dBm\n", lo, hi); return 1; }
    g_bpwr = PWR_DBM[pwr_idx((int)v)];
    changed();
    printf("OK bpwr=%d\n", g_bpwr);
    return 0;
}

static int cmd_pat(int argc, char **argv)
{
    int v = (argc >= 2) ? lookup(argv[1], PAT_NAMES, (int)N_PATS) : -1;
    if (v < 0) {
        printf("%s pat <", argc < 2 ? "usage:" : "ERR expected");
        for (size_t i = 0; i < N_PATS; i++) printf("%s%s", i ? "|" : "", PAT_NAMES[i]);
        printf(">\n");
        return 1;
    }
    g_pat = v;
    changed();
    printf("OK pat=%s\n", PAT_NAMES[v]);
    return 0;
}

static int cmd_len(int argc, char **argv)
{
    if (argc < 2) { printf("usage: len <0-255>\n"); return 1; }
    char *end;
    long v = strtol(argv[1], &end, 10);
    if (*end != '\0' || v < 0 || v > 255) { printf("ERR len out of range 0-255\n"); return 1; }
    g_len = (int)v;
    changed();
    printf("OK len=%d\n", (int)v);
    return 0;
}

void ble_register_commands(void)
{
    const esp_console_cmd_t cmds[] = {
        {.command = "ble",   .help = "BLE kind: adv (BT4) | ext (BT5) | dtm (test carrier)", .func = cmd_ble},
        {.command = "phy",   .help = "BLE PHY 1m|2m|s8|s2 (ext, dtm)",       .func = cmd_phy},
        {.command = "bch",   .help = "BLE DTM RF channel 0-39",              .func = cmd_bch},
        {.command = "advch", .help = "BLE advertising channels all|37,38,39", .func = cmd_advch},
        {.command = "itvl",  .help = "BLE advertising interval, ms",          .func = cmd_itvl},
        {.command = "bpwr",  .help = "BLE TX power, dBm",                     .func = cmd_bpwr},
        {.command = "pat",   .help = "BLE DTM payload pattern",               .func = cmd_pat},
        {.command = "len",   .help = "BLE DTM payload / ext adv data bytes",  .func = cmd_len},
    };
    for (size_t i = 0; i < sizeof(cmds) / sizeof(cmds[0]); i++) {
        ESP_ERROR_CHECK(esp_console_cmd_register(&cmds[i]));
    }
    uint8_t m[6];
    esp_read_mac(m, ESP_MAC_WIFI_STA);
    snprintf(s_name, sizeof(s_name), "BEESDR-TX-%02x%02x", m[4], m[5]);
}

int ble_status_snprint(char *buf, size_t n)
{
    uint8_t m[6] = {0};
    esp_read_mac(m, ESP_MAC_BT);
    char ach[12];
    advch_str(g_advch, ach, sizeof(ach));
    return snprintf(buf, n, " ble=%s bphy=%s coding=%s bch=%d advch=%s itvl=%d bpwr=%d bpwr_min=%d pat=%s len=%d"
           " ble_state=%s ble_err=%s name=%s bmac=%02x:%02x:%02x:%02x:%02x:%02x",
           KIND_NAMES[g_kind], PHY_NAMES[g_phy], (const char *)s_coding, g_bch, ach, g_itvl,
           g_bpwr, PWR_DBM[PWR_MIN_IDX], PAT_NAMES[g_pat], g_len,
           s_running ? (const char *)s_state : "off", s_err, s_name,
           m[0], m[1], m[2], m[3], m[4], m[5]);
}
