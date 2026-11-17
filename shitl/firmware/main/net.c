/*
 * net.c — the control link of ESP32-SHITL-Tx.
 *
 * The board is a WiFi station on a control network (credentials in NVS, set
 * with the `wifi` command over serial). While it is associated it offers:
 *
 *   HTTP, port 80
 *     GET  /status   the STATUS line (text/plain)
 *     POST /cmd      body = one command per line, run in order, stopping at the
 *                    first ERR; the reply is one line per command run.
 *     POST /ota      body = a firmware image (shitl_tx.bin). Written to the
 *                    inactive app slot, verified, then the board reboots into it.
 *   UDP broadcast, port 8762, once a second: the STATUS line. This is how the
 *   dashboard finds boards without being told their address.
 *
 * There is no authentication: anything on the control network can key the
 * transmitter or replace the firmware. Use it on a bench network.
 *
 * A job (shitl_tx.c) borrows the radio with net_offline_begin/end; the link is
 * dropped for the duration and re-established afterwards.
 *
 * Rollback: an image written over the air boots in "pending verify". It is
 * confirmed the moment it gets an IP address on the control network; if that
 * has not happened OTA_CONFIRM_S after boot, the board reboots and the
 * bootloader returns to the previous image. A bad update therefore cannot leave
 * the board unreachable.
 */

#include <string.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "esp_ota_ops.h"
#include "nvs.h"
#include "lwip/sockets.h"
#include "soc/soc_caps.h"

#include "app.h"

#define OTA_CONFIRM_S 90

static const char *TAG = "shitl-net";

static char s_ssid[33];
static char s_psk[65];
static volatile bool     s_want = true;       /* stay associated (false during a job) */
static volatile bool     s_assoc = false;
static volatile bool     s_online = false;    /* associated and has an address */
static volatile uint32_t s_ip = 0;            /* network byte order */
static volatile uint32_t s_drops = 0;
static volatile int64_t  s_apply_at = 0;      /* deferred `wifi` credential change */
static volatile bool     s_pending_verify = false;
static volatile bool     s_ota_busy = false;
static const char *volatile s_ota = "idle";   /* idle|writing|ok|failed */

const char *net_ssid(void) { return s_ssid; }

static void creds_load(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) return;
    size_t n = sizeof(s_ssid);
    if (nvs_get_str(h, "ssid", s_ssid, &n) != ESP_OK) s_ssid[0] = '\0';
    n = sizeof(s_psk);
    if (nvs_get_str(h, "psk", s_psk, &n) != ESP_OK) s_psk[0] = '\0';
    nvs_close(h);
}

void net_set_creds(const char *ssid, const char *psk)
{
    snprintf(s_ssid, sizeof(s_ssid), "%s", ssid);
    snprintf(s_psk, sizeof(s_psk), "%s", psk);
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_str(h, "ssid", s_ssid);
        nvs_set_str(h, "psk", s_psk);
        nvs_commit(h);
        nvs_close(h);
    }
    /* Applied from the housekeeping task shortly, so an HTTP reply gets out first. */
    s_apply_at = esp_timer_get_time() + 300000;
}

static void apply_creds(void)
{
    wifi_config_t wc = {0};
    /* Fixed-size fields; a full 32-byte SSID is legitimately not NUL-terminated. */
    memcpy(wc.sta.ssid, s_ssid, strnlen(s_ssid, sizeof(wc.sta.ssid)));
    memcpy(wc.sta.password, s_psk, strnlen(s_psk, sizeof(wc.sta.password) - 1));
    wc.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;       /* strongest AP of that name */
    wc.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
    wc.sta.pmf_cfg.capable = true;
    esp_wifi_set_config(WIFI_IF_STA, &wc);
}

static void on_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        if (s_want && s_ssid[0]) esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_CONNECTED) {
        s_assoc = true;
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_online) s_drops++;
        s_assoc = false; s_online = false; s_ip = 0;
        if (s_want && s_ssid[0]) esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *ev = (ip_event_got_ip_t *)data;
        s_ip = ev->ip_info.ip.addr;
        s_online = true;
        ESP_LOGI(TAG, "online: " IPSTR, IP2STR(&ev->ip_info.ip));
        if (s_pending_verify) {
            esp_ota_mark_app_valid_cancel_rollback();
            s_pending_verify = false;
            ESP_LOGI(TAG, "new firmware confirmed");
        }
    }
}

void net_offline_begin(bool stop_wifi_radio)
{
    s_want = false;
    esp_wifi_disconnect();
    for (int i = 0; i < 100 && s_assoc; i++) vTaskDelay(pdMS_TO_TICKS(10));
    vTaskDelay(pdMS_TO_TICKS(50));
    if (stop_wifi_radio) esp_wifi_stop();
}

void net_offline_end(bool wifi_radio_stopped)
{
    s_want = true;
    if (wifi_radio_stopped) {
        esp_wifi_start();                     /* STA_START -> connect */
        esp_wifi_set_ps(WIFI_PS_NONE);
    }
    esp_wifi_set_max_tx_power(80);            /* the link gets full power back */
    if (!wifi_radio_stopped && s_ssid[0]) esp_wifi_connect();
}

int net_status_snprint(char *buf, size_t n)
{
    char ap[33];
    snprintf(ap, sizeof(ap), "%s", s_ssid[0] ? s_ssid : "-");
    for (char *c = ap; *c; c++) if (*c == ' ') *c = '_';   /* keep key=value parseable */
    int rssi = 0, apch = 0;
    wifi_ap_record_t rec;
    if (s_online && esp_wifi_sta_get_ap_info(&rec) == ESP_OK) { rssi = rec.rssi; apch = rec.primary; }
    uint32_t ip = s_ip;
    return snprintf(buf, n, " net=%s ap=%s ip=%u.%u.%u.%u rssi=%d apch=%d drops=%u ota=%s",
                    !s_ssid[0] ? "unprovisioned" : s_online ? "online" : s_want ? "joining" : "offline",
                    ap, (unsigned)(ip & 0xff), (unsigned)((ip >> 8) & 0xff),
                    (unsigned)((ip >> 16) & 0xff), (unsigned)((ip >> 24) & 0xff),
                    rssi, apch, (unsigned)s_drops, (const char *)s_ota);
}

/* ------------------------------------------------------------------- HTTP --- */

static char s_line[1280];

static esp_err_t h_status(httpd_req_t *r)
{
    static char buf[1280];
    int k = status_snprint(buf, sizeof(buf) - 1);
    buf[k++] = '\n';
    httpd_resp_set_type(r, "text/plain");
    return httpd_resp_send(r, buf, k);
}

static esp_err_t h_cmd(httpd_req_t *r)
{
    static char body[1024], out[2560];
    int len = (int)r->content_len;
    if (len <= 0 || len >= (int)sizeof(body)) {
        return httpd_resp_send_err(r, HTTPD_400_BAD_REQUEST, "body must be 1-1023 bytes of command lines");
    }
    int got = 0;
    while (got < len) {
        int k = httpd_req_recv(r, body + got, len - got);
        if (k == HTTPD_SOCK_ERR_TIMEOUT) continue;
        if (k <= 0) return ESP_FAIL;
        got += k;
    }
    body[len] = '\0';
    size_t o = 0;
    out[0] = '\0';
    char *save = NULL;
    for (char *ln = strtok_r(body, "\r\n", &save); ln; ln = strtok_r(NULL, "\r\n", &save)) {
        if (o + 64 >= sizeof(out)) break;
        int rc = cmd_exec_line(ln, out + o, sizeof(out) - o);
        o += strlen(out + o);
        if (rc != 0) break;
    }
    httpd_resp_set_type(r, "text/plain");
    return httpd_resp_send(r, out, (ssize_t)o);
}

static void reboot_cb(void *arg) { esp_restart(); }

static esp_err_t ota_fail(httpd_req_t *r, const char *why, esp_err_t e)
{
    char msg[96];
    snprintf(msg, sizeof(msg), "ERR ota %s: %s\n", why, esp_err_to_name(e));
    ESP_LOGE(TAG, "%s", msg);
    s_ota = "failed";
    s_ota_busy = false;
    job_lock_for_ota(false);
    httpd_resp_set_status(r, "500 Internal Server Error");
    httpd_resp_set_type(r, "text/plain");
    httpd_resp_send(r, msg, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t h_ota(httpd_req_t *r)
{
    static char chunk[4096];
    int len = (int)r->content_len;
    const esp_partition_t *dst = esp_ota_get_next_update_partition(NULL);
    httpd_resp_set_type(r, "text/plain");
    if (!dst) {
        httpd_resp_set_status(r, "500 Internal Server Error");
        return httpd_resp_sendstr(r, "ERR ota: no second app slot in the partition table\n");
    }
    if (len < 0x10000 || len > (int)dst->size) {
        httpd_resp_set_status(r, "400 Bad Request");
        return httpd_resp_sendstr(r, "ERR ota: image size out of range for the app slot\n");
    }
    if (s_ota_busy || !job_lock_for_ota(true)) {
        httpd_resp_set_status(r, "409 Conflict");
        return httpd_resp_sendstr(r, "ERR ota: busy (a job or another update is in progress)\n");
    }
    s_ota_busy = true;
    s_ota = "writing";
    ESP_LOGI(TAG, "ota: %d bytes -> %s", len, dst->label);

    esp_ota_handle_t h;
    esp_err_t e = esp_ota_begin(dst, (size_t)len, &h);
    if (e != ESP_OK) return ota_fail(r, "begin", e);
    int got = 0, stalls = 0;
    while (got < len) {
        int want = len - got < (int)sizeof(chunk) ? len - got : (int)sizeof(chunk);
        int k = httpd_req_recv(r, chunk, want);
        if (k == HTTPD_SOCK_ERR_TIMEOUT && ++stalls < 6) continue;
        if (k <= 0) { esp_ota_abort(h); return ota_fail(r, "receive", ESP_ERR_TIMEOUT); }
        stalls = 0;
        e = esp_ota_write(h, chunk, (size_t)k);
        if (e != ESP_OK) { esp_ota_abort(h); return ota_fail(r, "write", e); }
        got += k;
    }
    e = esp_ota_end(h);                         /* verifies checksum, hash and chip type */
    if (e != ESP_OK) return ota_fail(r, "verify", e);
    e = esp_ota_set_boot_partition(dst);
    if (e != ESP_OK) return ota_fail(r, "set_boot", e);

    s_ota = "ok";
    snprintf(s_line, sizeof(s_line), "OK ota bytes=%d part=%s rebooting\n", got, dst->label);
    httpd_resp_sendstr(r, s_line);
    static esp_timer_handle_t t;
    const esp_timer_create_args_t ta = {.callback = reboot_cb, .name = "reboot"};
    if (esp_timer_create(&ta, &t) == ESP_OK) esp_timer_start_once(t, 800000);
    else esp_restart();
    return ESP_OK;
}

static void http_start(void)
{
    httpd_config_t c = HTTPD_DEFAULT_CONFIG();
    c.server_port = HTTP_PORT;
    c.stack_size = 8192;
    c.lru_purge_enable = true;
    c.recv_wait_timeout = 10;
    httpd_handle_t srv = NULL;
    if (httpd_start(&srv, &c) != ESP_OK) { ESP_LOGE(TAG, "http server failed to start"); return; }
    const httpd_uri_t u[] = {
        {.uri = "/status", .method = HTTP_GET,  .handler = h_status},
        {.uri = "/cmd",    .method = HTTP_POST, .handler = h_cmd},
        {.uri = "/ota",    .method = HTTP_POST, .handler = h_ota},
    };
    for (size_t i = 0; i < sizeof(u) / sizeof(u[0]); i++) httpd_register_uri_handler(srv, &u[i]);
}

/* ----------------------------------------------- announce + housekeeping --- */

static void net_task(void *arg)
{
    static char buf[1280];
    int sock = -1;
    int tick = 0;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(200));

        int64_t at = s_apply_at;
        if (at && esp_timer_get_time() >= at) {
            s_apply_at = 0;
            apply_creds();
            if (s_want) {
                esp_wifi_disconnect();        /* the DISCONNECTED handler reconnects */
                if (!s_assoc && s_ssid[0]) esp_wifi_connect();
            }
        }

        if (s_pending_verify && esp_timer_get_time() > (int64_t)OTA_CONFIRM_S * 1000000 && job_idle()) {
            ESP_LOGE(TAG, "new firmware never reached the control network: rolling back");
            esp_ota_mark_app_invalid_rollback_and_reboot();
        }

        if (++tick % 5 != 0 || !s_online) continue;
        if (sock < 0) {
            sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
            if (sock < 0) continue;
            int on = 1;
            setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &on, sizeof(on));
        }
        int k = status_snprint(buf, sizeof(buf));
        struct sockaddr_in to = {
            .sin_family = AF_INET, .sin_port = htons(ANNOUNCE_PORT),
            .sin_addr.s_addr = htonl(INADDR_BROADCAST),
        };
        if (sendto(sock, buf, k, 0, (struct sockaddr *)&to, sizeof(to)) < 0) {
            close(sock);
            sock = -1;
        }
    }
}

void net_start(void)
{
    creds_load();

    esp_ota_img_states_t st;
    if (esp_ota_get_state_partition(esp_ota_get_running_partition(), &st) == ESP_OK
        && st == ESP_OTA_IMG_PENDING_VERIFY) {
        if (s_ssid[0]) {
            s_pending_verify = true;
        } else {
            esp_ota_mark_app_valid_cancel_rollback();   /* no network to prove itself on */
        }
    }

    esp_netif_t *nif = esp_netif_create_default_wifi_sta();
    uint8_t m[6];
    esp_read_mac(m, ESP_MAC_WIFI_STA);
    char host[20];
    snprintf(host, sizeof(host), "shitltx-%02x%02x", m[4], m[5]);
    esp_netif_set_hostname(nif, host);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, on_event, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, on_event, NULL));

    /* Manual country policy over the full 1-14 span so injection is not clamped
     * on the upper channels (12-14 are region-restricted: deliberate lab use). */
    wifi_country_t country = {
        .cc = "01", .schan = 1, .nchan = 14, .policy = WIFI_COUNTRY_POLICY_MANUAL,
    };
    ESP_ERROR_CHECK(esp_wifi_set_country(&country));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA,
        WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N
#if SOC_WIFI_HE_SUPPORT
        | WIFI_PROTOCOL_11AX
#endif
        ));
    apply_creds();
    ESP_ERROR_CHECK(esp_wifi_start());
    esp_wifi_set_ps(WIFI_PS_NONE);

    http_start();
    xTaskCreate(net_task, "net", 4096, NULL, 4, NULL);
    ESP_LOGI(TAG, "control network: %s", s_ssid[0] ? s_ssid : "(not set - use: wifi <ssid> <password>)");
}
