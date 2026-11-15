/*
 * ble_tx.h — Bluetooth LE transmit mode for the BEESDR ESP32-Tx firmware.
 * See ble_tx.c for what it puts on air. beacon_tx.c owns boot, the console,
 * NVS and the settings shared by both radios (tx on/off, duty).
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include "nvs.h"

/* Shared with beacon_tx.c */
extern volatile bool g_tx_on;    /* tx on|off — gates both radios          */
extern volatile int  g_duty;     /* 0..100 — WiFi injection and BLE DTM    */
void cfg_save(void);             /* persist every setting to NVS           */

void ble_cfg_load(nvs_handle_t h);
void ble_cfg_save(nvs_handle_t h);
void ble_register_commands(void);

/* Bring up the BLE stack and start transmitting. Call once, in BLE mode only;
 * the WiFi stack must not be running (single radio, one stack per boot). */
void ble_tx_start(void);

/* A shared setting (tx / duty) or a BLE one changed: re-apply it on air.
 * Harmless no-op in WiFi mode. */
void ble_cfg_changed(void);

/* Format the BLE key=value fields of the STATUS line (leading space, no \n). */
int ble_status_snprint(char *buf, size_t n);
