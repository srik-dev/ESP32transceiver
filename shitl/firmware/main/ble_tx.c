/*
 * ble_tx.c — Bluetooth LE transmission for ESP32-SHITL-Tx.
 *
 * Same three kinds as ESP32-Tx, selected with `ble <adv|ext|dtm>`:
 *
 *   adv  Bluetooth 4 legacy advertising, ADV_NONCONN_IND on 37/38/39, LE 1M.
 *   ext  Bluetooth 5 extended advertising. `phy`: 1m | 2m (secondary only) |
 *        s8 | s2 (Coded on both). `len` pads the advertising data.
 *   dtm  Direct Test Mode: back-to-back test packets on ONE RF channel 0-39
 *        (2402 + 2k MHz), any PHY, chosen payload pattern and length. `duty`
 *        gates it in 100 ms windows. The steady emitter for SDR / AoA work.
 *
 * Unlike ESP32-Tx there is no reboot between WiFi and BLE. A BLE job stops the
 * WiFi radio (net_offline_begin), brings NimBLE up (ble_tx_up), transmits until
 * the job's deadline (ble_tx_run) and tears NimBLE down again (ble_tx_down), so
 * each radio has the antenna to itself while it is in use.
 *
 * Commands (saved to NVS): ble, phy, bch, advch, itvl, bpwr, pat, len.
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "esp_bt.h"
#include "sdkconfig.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/util/util.h"

#include "app.h"

static const char *TAG = "shitl-ble";

enum { KIND_ADV, KIND_EXT, KIND_DTM, N_KINDS };
static const char *const KIND_NAMES[N_KINDS] = {"adv", "ext", "dtm"};

enum { PHY_1M, PHY_2M, PHY_S8, PHY_S2, N_PHYS };
static const char *const PHY_NAMES[N_PHYS] = {"1m", "2m", "s8", "s2"};

/* DTM payload patterns, indexed by the HCI LE Transmitter Test payload code. */
static const char *const PAT_NAMES[] = {
    "prbs9", "11110000", "10101010", "prbs15", "ones", "zeros", "00001111", "01010101",
};
#define N_PATS (sizeof(PAT_NAMES) / sizeof(PAT_NAMES[0]))

/* esp_power_level_t index -> dBm. The C6 controller starts at -15 dBm (index 3). */
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

static volatile int g_kind   = KIND_ADV;
static volatile int g_phy    = PHY_1M;
static volatile int g_bch    = 19;     /* DTM RF channel index: 2440 MHz */
static volatile int g_advch  = 0x07;   /* bit0=37 bit1=38 bit2=39        */
static volatile int g_itvl   = 20;     /* ms                             */
static volatile int g_bpwr   = 9;      /* dBm, one of PWR_DBM[]          */
static volatile int g_pat    = 0;
static volatile int g_len    = 37;

static volatile bool     s_synced = false;
static volatile bool     s_up = false;
static const char *volatile s_state = "off";
static const char *volatile s_coding = "-";
static char              s_err[32] = "0";
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

/* Flags, Complete Local Name, and (extended only) a manufacturer-specific AD
 * (test company ID 0xFFFF) padding the total to `len`. */
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

static int adv_start(int kind, int phy, int advch, int itvl_ms, int pwr, int len)
{
    struct ble_gap_ext_adv_params p;
    memset(&p, 0, sizeof(p));
    p.legacy_pdu = (kind == KIND_ADV);          /* ADV_NONCONN_IND */
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
        /* Set Extended Advertising Parameters v2: 3 = S=2 required, 4 = S=8
         * required. Only this command can pick the coding. */
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
        if (rc == 0 && coded) s_coding = "ctrl";
    }
    if (rc != 0) { set_err("adv_cfg", rc); return rc; }
    s_adv_cfgd = true;

    uint8_t data[EXT_DATA_MAX];
    int n = build_adv_data(data, kind, len);
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

/* Sleep up to ms, but not past end_us and not through a cancel. */
static void nap(int ms, int64_t end_us)
{
    int64_t until = esp_timer_get_time() + (int64_t)ms * 1000;
    if (until > end_us) until = end_us;
    while (!job_cancelled()) {
        int64_t left = until - esp_timer_get_time();
        if (left <= 0) break;
        int step = left > 20000 ? 20 : (int)((left + 999) / 1000);
        vTaskDelay(pdMS_TO_TICKS(step > 0 ? step : 1));
    }
}

int ble_tx_run(int64_t end_us)
{
    const int WINDOW_MS = 100;
    if (!s_up || !s_synced) { s_state = "error"; return -1; }

    int kind = g_kind, phy = g_phy, bch = g_bch, pat = g_pat, len = g_len;
    int pwr = g_bpwr, duty = g_duty;
    int rc = 0;
    s_coding = "-";
    apply_power(pwr);
    ESP_LOGI(TAG, "ble job: kind=%s phy=%s pwr=%d", KIND_NAMES[kind], PHY_NAMES[phy], pwr);

    if (kind != KIND_DTM) {
        rc = adv_start(kind, phy, g_advch, g_itvl, pwr, len);
        if (rc == 0) {
            s_state = "adv";
            nap(INT32_MAX / 2000, end_us);
        }
    } else {
        s_state = "dtm";
        if (phy == PHY_S8 || phy == PHY_S2) s_coding = PHY_NAMES[phy];
        int on_ms = WINDOW_MS * duty / 100;
        while (!job_cancelled() && esp_timer_get_time() < end_us) {
            if (duty >= 100) {
                if (!s_dtm_on && (rc = dtm_start(bch, len, pat, phy)) != 0) break;
                nap(WINDOW_MS, end_us);
            } else if (duty <= 0) {
                nap(WINDOW_MS, end_us);
            } else {
                if ((rc = dtm_start(bch, len, pat, phy)) != 0) break;
                nap(on_ms, end_us);
                ble_gap_dtm_stop();
                s_dtm_on = false;
                nap(WINDOW_MS - on_ms, end_us);
            }
        }
    }
    ble_stop();
    s_state = rc == 0 ? "idle" : "error";
    return rc;
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

void ble_tx_init(void)
{
    uint8_t m[6];
    esp_read_mac(m, ESP_MAC_WIFI_STA);
    snprintf(s_name, sizeof(s_name), "BEESDR-TX-%02x%02x", m[4], m[5]);
    s_state = "idle";
}

/* The BLE controller and host exist only for the length of a BLE job, so each
 * radio stack owns the antenna outright while it is in use and WiFi is never
 * scheduled around an idle BLE controller. */
int ble_tx_up(void)
{
    s_synced = false;
    strcpy(s_err, "0");
    esp_err_t e = nimble_port_init();
    if (e != ESP_OK) {
        set_err("nimble_init", (int)e);
        s_state = "error";
        return -1;
    }
    ble_hs_cfg.sync_cb = on_sync;
    ble_hs_cfg.reset_cb = on_reset;
    nimble_port_freertos_init(host_task);
    s_up = true;
    for (int i = 0; i < 150 && !s_synced; i++) vTaskDelay(pdMS_TO_TICKS(20));
    if (!s_synced) { set_err("sync", -1); s_state = "error"; return -1; }
    return 0;
}

/* ESP-IDF 5.5: ble_hs_deinit() calls ble_sm_deinit() unconditionally, but the
 * security manager is not compiled in a broadcaster-only build (no connection
 * roles), so the link fails. There is nothing to tear down; weak, so a build
 * that does include the real one uses it. */
__attribute__((weak)) void ble_sm_deinit(void) {}

void ble_tx_down(void)
{
    if (!s_up) return;
    ble_stop();
    if (nimble_port_stop() == 0) nimble_port_deinit();
    s_up = false;
    s_synced = false;
}

/* ---------------------------------------------------------------- console --- */

static int lookup(const char *s, const char *const *names, int n)
{
    for (int i = 0; i < n; i++) {
        if (strcmp(s, names[i]) == 0) return i;
    }
    return -1;
}

int cmd_ble(int argc, char **argv)
{
    int v = (argc >= 2) ? lookup(argv[1], KIND_NAMES, N_KINDS) : -1;
    if (v < 0) { rpl("ERR expected ble <adv|ext|dtm>\n"); return 1; }
    g_kind = v;
    cfg_save();
    rpl("OK ble=%s\n", KIND_NAMES[v]);
    return 0;
}

int cmd_phy(int argc, char **argv)
{
    int v = (argc >= 2) ? lookup(argv[1], PHY_NAMES, N_PHYS) : -1;
    if (v < 0) { rpl("ERR expected phy <1m|2m|s8|s2>\n"); return 1; }
    g_phy = v;
    cfg_save();
    rpl("OK phy=%s\n", PHY_NAMES[v]);
    return 0;
}

int cmd_bch(int argc, char **argv)
{
    char *end = NULL;
    long v = argc >= 2 ? strtol(argv[1], &end, 10) : -1;
    if (argc < 2 || *end != '\0' || v < 0 || v > 39) { rpl("ERR bch out of range 0-39\n"); return 1; }
    g_bch = (int)v;
    cfg_save();
    rpl("OK bch=%d freq=%d\n", (int)v, 2402 + 2 * (int)v);
    return 0;
}

int cmd_advch(int argc, char **argv)
{
    if (argc < 2) { rpl("ERR usage: advch <all|37,38,39 subset>\n"); return 1; }
    int mask = 0;
    if (strcmp(argv[1], "all") == 0) {
        mask = 7;
    } else {
        char tmp[16];
        char *save = NULL;
        snprintf(tmp, sizeof(tmp), "%s", argv[1]);
        for (char *t = strtok_r(tmp, ",", &save); t; t = strtok_r(NULL, ",", &save)) {
            int c = atoi(t);
            if (c < 37 || c > 39) { rpl("ERR advch takes 37, 38, 39 or all\n"); return 1; }
            mask |= 1 << (c - 37);
        }
    }
    if (mask == 0) { rpl("ERR advch needs at least one channel\n"); return 1; }
    g_advch = mask;
    cfg_save();
    char s[12];
    advch_str(mask, s, sizeof(s));
    rpl("OK advch=%s\n", s);
    return 0;
}

int cmd_itvl(int argc, char **argv)
{
    int v = argc >= 2 ? atoi(argv[1]) : 0;
    if (v < ITVL_MIN_MS || v > ITVL_MAX_MS) {
        rpl("ERR itvl out of range %d-%d ms\n", ITVL_MIN_MS, ITVL_MAX_MS);
        return 1;
    }
    g_itvl = v;
    cfg_save();
    rpl("OK itvl=%d\n", v);
    return 0;
}

int cmd_bpwr(int argc, char **argv)
{
    int lo = PWR_DBM[PWR_MIN_IDX], hi = PWR_DBM[N_PWR - 1];
    char *end = NULL;
    long v = argc >= 2 ? strtol(argv[1], &end, 10) : 0;
    if (argc < 2 || *end != '\0' || v < lo || v > hi) { rpl("ERR bpwr out of range %d..%d dBm\n", lo, hi); return 1; }
    g_bpwr = PWR_DBM[pwr_idx((int)v)];
    cfg_save();
    rpl("OK bpwr=%d\n", g_bpwr);
    return 0;
}

int cmd_pat(int argc, char **argv)
{
    int v = (argc >= 2) ? lookup(argv[1], PAT_NAMES, (int)N_PATS) : -1;
    if (v < 0) {
        rpl("ERR expected pat <");
        for (size_t i = 0; i < N_PATS; i++) rpl("%s%s", i ? "|" : "", PAT_NAMES[i]);
        rpl(">\n");
        return 1;
    }
    g_pat = v;
    cfg_save();
    rpl("OK pat=%s\n", PAT_NAMES[v]);
    return 0;
}

int cmd_len(int argc, char **argv)
{
    char *end = NULL;
    long v = argc >= 2 ? strtol(argv[1], &end, 10) : -1;
    if (argc < 2 || *end != '\0' || v < 0 || v > 255) { rpl("ERR len out of range 0-255\n"); return 1; }
    g_len = (int)v;
    cfg_save();
    rpl("OK len=%d\n", (int)v);
    return 0;
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
           (const char *)s_state, s_err, s_name,
           m[0], m[1], m[2], m[3], m[4], m[5]);
}
