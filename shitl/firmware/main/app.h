/*
 * app.h — shared declarations for the ESP32-SHITL-Tx firmware.
 *
 *   shitl_tx.c  settings, command set, the timed job runner, WiFi injection
 *   net.c       station link to the control network, HTTP API (commands and
 *               firmware update), UDP announce
 *   ble_tx.c    Bluetooth LE advertising / DTM
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "nvs.h"

#define NVS_NS        "shitltx"
#define HTTP_PORT     80
#define ANNOUNCE_PORT 8762      /* UDP broadcast of the STATUS line, 1 Hz */

/* ---- shitl_tx.c ---- */
extern volatile int g_duty;                 /* 0..100, WiFi injection and BLE DTM */
void cfg_save(void);
void rpl(const char *fmt, ...) __attribute__((format(printf, 1, 2)));  /* append to the command reply */
/* Run one command line. `line` is modified. The reply ("OK ..." / "ERR ...",
 * newline-terminated) is written to out. Returns 0 on OK. Thread-safe. */
int  cmd_exec_line(char *line, char *out, size_t n);
int  status_snprint(char *buf, size_t n);   /* the whole STATUS line, no newline */
bool job_cancelled(void);
bool job_idle(void);
/* Firmware update in progress: refuses new jobs. Returns false if a job is
 * pending or running (the update must then be refused). */
bool job_lock_for_ota(bool lock);

/* ---- net.c ---- */
void net_start(void);                        /* WiFi stack, station link, HTTP, announce */
void net_set_creds(const char *ssid, const char *psk);   /* saved; applied ~0.3 s later */
const char *net_ssid(void);
/* Leave the control network so the radio is free for a job, and come back. */
void net_offline_begin(bool stop_wifi_radio);
void net_offline_end(bool wifi_radio_stopped);
int  net_status_snprint(char *buf, size_t n);

/* ---- ble_tx.c ---- */
void ble_cfg_load(nvs_handle_t h);
void ble_cfg_save(nvs_handle_t h);
void ble_tx_init(void);
int  ble_tx_up(void);                        /* start NimBLE for a job (WiFi radio stopped) */
void ble_tx_down(void);
int  ble_tx_run(int64_t end_us);             /* transmit until end_us or job_cancelled() */
int  ble_status_snprint(char *buf, size_t n);
int  cmd_ble(int argc, char **argv);
int  cmd_phy(int argc, char **argv);
int  cmd_bch(int argc, char **argv);
int  cmd_advch(int argc, char **argv);
int  cmd_itvl(int argc, char **argv);
int  cmd_bpwr(int argc, char **argv);
int  cmd_pat(int argc, char **argv);
int  cmd_len(int argc, char **argv);
