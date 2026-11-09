# ESP32Tx — ESP32-C6 WiFi channel transmitters

Firmware and host tooling that turn one or more **ESP32-C6** dev boards into
controllable 2.4 GHz WiFi emitters, each parked on a channel you choose, with a
**web dashboard that runs continuously on the Mac** so the boards are always
controllable without launching anything.

They exist as **signal sources** for direction-finding / AoA work: a repeatable,
cheap, channel-selectable transmitter an SDR can point at, instead of depending on
whatever ambient ISM traffic happens to be in the room.

---

## At a glance

| | |
|---|---|
| **Dashboard URL** | **http://127.0.0.1:8760/** |
| **Port** | **8760** (TCP; `--port` to change) |
| **Always running?** | **Yes** — macOS LaunchAgent, starts at login, `KeepAlive` restarts it |
| **Service label** | `com.beesdr.esp32c6-dashboard` |
| **Runs from** | `~/Library/Application Support/BEESDR/ESP32C6-dashboard/` (**not** this folder — see [Why the service runs from a copy](#why-the-service-runs-from-a-copy)) |
| **Logs** | `~/Library/Logs/beesdr-esp32c6-dashboard{,.error}.log` |
| **Runtime deps** | System `python3` only — stdlib, no venv, no pyserial |
| **Toolchain** | Shared ESP-IDF **v5.5.5** at `~/esp/esp-idf` |
| **Target** | `esp32c6` |

Plug a C6 into USB and it appears in the dashboard within a couple of seconds —
there is nothing to start.

```bash
open http://127.0.0.1:8760/          # the UI
dashboard/deploy.sh --status         # is the service loaded and answering?
```

---

## Scope

**What this is.** A lab-bench RF signal generator built out of commodity ESP32-C6
boards, plus everything needed to drive it: firmware, a serial control tool, and an
always-on web UI. One firmware image serves every board; boards are differentiated
at runtime by the channel you assign, so a bag of identical boards becomes a
multi-channel emitter set.

**What it is deliberately not:**

- **Not a WiFi client or AP.** The boards never associate. They inject raw 802.11
  beacon frames and nothing else.
- **Not a receiver.** Despite the repository name, this is transmit-only. There is
  no capture, demodulation or sniffing path.
- **Not BLE.** The C6 supports it; this firmware does not use it.
- **Not remotely controlled over WiFi.** Control is **USB serial only**, by
  necessity — the C6 is a single-radio, 2.4 GHz-only part and cannot stay
  associated to an access point while injecting on a channel of your choosing. The
  dashboard is a web UI *on the Mac* that bridges HTTP to the USB serial ports; the
  boards themselves are never on the network.
- **Not a general ESP-IDF template.** It is one application, pinned to one target.

**Intended use.** Deliberate, authorized injection on a bench or in a shielded
setup. See [Notes / gotchas](#notes--gotchas) about channels 12–14.

---

## Repository layout

```
ESP32Tx/
├── README.md                  this file
├── .gitignore                 ignores IDF build output (see "Generated" below)
│
├── firmware/                  ESP-IDF project, target esp32c6
│   ├── CMakeLists.txt         project file; finds IDF via $ENV{IDF_PATH}
│   ├── sdkconfig.defaults     the only tracked config — sdkconfig is derived
│   └── main/
│       ├── CMakeLists.txt     component registration
│       ├── idf_component.yml  managed deps (espressif/led_strip)
│       └── beacon_tx.c        the whole firmware: TX loop + serial command REPL
│
├── tools/                     host-side, run from the repo root
│   ├── idf_env.sh             SOURCE this to activate the shared ESP-IDF
│   ├── setup_idf.sh           one-time IDF install (reference / fallback)
│   ├── flash.sh               build once, flash one or many boards
│   └── esp32c6_ctl.py         enumerate boards, assign channels (stdlib only)
│
└── dashboard/                 the always-on web UI, port 8760
    ├── serve.py               stdlib HTTP server + USB-serial bridge
    ├── index.html             the single-page UI
    ├── dashboard.sh           run it in the foreground (development)
    └── deploy.sh              sync to the background service + restart it
```

**Generated, not tracked** — `.gitignore` covers all of it. ESP-IDF regenerates
`build/` from source in about 30 s, so none of it belongs in git:

| Path | What it is |
|---|---|
| `firmware/build/` | ~215 MB of CMake/ninja output, incl. `beesdr_wifi_tx.bin` |
| `firmware/sdkconfig` | derived from `sdkconfig.defaults` on first build |
| `firmware/managed_components/` | fetched by the IDF component manager |
| `firmware/dependencies.lock` | resolved component versions |

⚠️ **A build tree is pinned to its absolute path.** CMake records its own source
and binary directory, so *moving this project invalidates `build/`* — the next
`idf.py build` fails on the stale cache rather than on your code. After any move,
delete `firmware/build/` and rebuild.

---

## Hardware

Any ESP32-C6 dev board. Two USB wiring styles exist and the tooling handles both,
but they are worth telling apart because they enumerate differently:

| Path | Enumerates as | Console on |
|---|---|---|
| Native USB-Serial-JTAG (VID `0x303a`) | `/dev/cu.usbmodem*` | the native peripheral |
| USB-UART bridge, e.g. WCH CH343 (VID `0x1a86`) | `/dev/cu.usbmodem*`, `usbserial*`, `wchusbserial*` | UART0 |

The boards in use here are the **CH343 bridge** variety, with the firmware console
on **UART0**. `flash.sh`, `esp32c6_ctl.py` and `serve.py` all glob every pattern
above and filter out the macOS Bluetooth and `debug-console` pseudo-ports, so
either style is picked up automatically.

Whichever the board, one USB cable carries the log, the flash channel and the
interactive REPL on the **same** port — no BOOT/RESET button dance for normal
flashing.

**Identifying boards.** The `/dev` name can shift between plug-ins; the **MAC is
the stable identity**, and the dashboard shows it. Several boards can share one Mac
port through a USB hub.

**Gotcha:** a charge-only cable lights the board's LED but produces no `/dev/cu.*`
entry at all. Use a data cable.

---

## Prerequisites

**ESP-IDF v5.1+, target esp32c6 — installed at `~/esp/esp-idf` (currently
v5.5.5)**, with the RISC-V toolchain under `~/.espressif`.

That install is **shared by every ESP project on this machine**. Nothing is
vendored into this repo, and new projects should point at the same copy rather than
cloning their own (~2 GB each).

`idf.py` is not on `PATH` by default. Activate it:

```bash
. tools/idf_env.sh            # IDF_PATH -> $IDF_DIR -> ~/esp/esp-idf, first hit wins
```

Override the location with `IDF_DIR=/path/to/esp-idf`. If IDF is missing entirely,
`tools/setup_idf.sh` installs it to the shared path.

`tools/flash.sh` sources `idf_env.sh` itself, so it works from a cold shell.

The **host tools need nothing but the system `python3`** — `esp32c6_ctl.py` and the
dashboard talk to the tty through stdlib `termios`. No venv, no pip, no pyserial.

---

## Build & flash

From `firmware/`, with IDF activated:

```bash
. ../tools/idf_env.sh
idf.py set-target esp32c6
idf.py build
idf.py -p /dev/cu.usbmodemXXXX flash monitor
```

Or flash every plugged-in board in one shot, from a cold shell:

```bash
tools/flash.sh                                  # every detected port
tools/flash.sh /dev/cu.usbmodem101              # one specific port
tools/flash.sh --monitor /dev/cu.usbmodem101    # flash, then open the monitor
```

**Stop the dashboard service first** — one process per serial port, and the
always-on service holds them.

---

## Assign channels to boards

One image serves every board; the channel you assign after flashing is what
differentiates them. Plug in all the boards, then:

```bash
# What's connected, and each board's current state:
tools/esp32c6_ctl.py list

# Round-robin channels 1, 6, 11 across however many boards are present,
# 100% duty, 20 dBm, and start transmitting:
tools/esp32c6_ctl.py assign --channels 1,6,11 --duty 100 --pwr 20

# Bursty emitter (20% on-air), for exercising a receiver's burst gate:
tools/esp32c6_ctl.py assign --channels 6 --duty 20
```

`assign` walks the detected ports in order and hands out the channel list
round-robin — 5 boards + `--channels 1,6,11` → `1,6,11,1,6`. To pin specific boards
to specific ports, pass `--port` (repeatable) in the order you want.

---

## Runtime control (serial commands)

The firmware runs a line REPL (`beesdr-tx>`) on the console. Type these in
`idf.py monitor`, send them with `esp32c6_ctl.py send`, or use the dashboard:

| Command | Effect |
|---|---|
| `ch <1-14>` | Set channel. SSID and DS-parameter IE track it. |
| `duty <0-100>` | On-air percent of a 100 ms window. 100 = continuous, lower = bursty. |
| `pwr <2-20>` | Max TX power, dBm. |
| `rate <name>` | PHY rate — this also picks the 802.11 standard (table below). `6m` default. |
| `tx <on\|off>` | Enable / pause injection. |
| `status` | Print MAC, channel, duty, power, rate, `phy`, `rate_err`, tx state, frame/error counters. |

### PHY rates and 802.11 standards

| Standard | `rate` names | PHY | Beacon advertises |
|---|---|---|---|
| **802.11b** | `1m 2m 5m 11m` | DSSS/CCK, ~22 MHz | b/g rates |
| **802.11g** | `6m 9m 12m 18m 24m 36m 48m 54m` | OFDM, 20 MHz | b/g rates |
| **802.11n** (Wi-Fi 4) | `mcs0`..`mcs7` | HT20, 800 ns GI | + HT Capabilities / Operation |
| **802.11ax** (Wi-Fi 6) | `he-mcs0`..`he-mcs9` | HE20 SU, 1.6 µs GI | + HT **and** HE Capabilities / Operation |

**802.11ac is not possible**: it is 5 GHz-only and the C6 is a 2.4 GHz radio.

Rates are applied with `esp_wifi_config_80211_tx()` (PHY mode + rate). The older
`esp_wifi_config_80211_tx_rate()` cannot select HE. ⚠️ **Do not wrap the call in
`esp_wifi_stop()`/`start()`**, even though the IDF header says "call before
`esp_wifi_start()`". `stop()` tears down the interface state the library reads, so
every rate then returns `ESP_FAIL` (seen on hardware). Calling it on the running
interface works. A failure shows up as `rate_err=<IDF error>` in `status` and in red
next to the PHY selector in the dashboard.

### Settings persist across power cycles

Every accepted `ch` / `duty` / `pwr` / `rate` / `tx` / `led` command is saved to NVS
flash right away and restored at boot. **A board unplugged from the Mac and powered
from any USB supply comes back up transmitting exactly what it was last set to**,
with no host involved. The dashboard (or `esp32c6_ctl.py`) is only needed to *change*
settings. Erasing flash (`idf.py erase-flash`) returns a board to the defaults
(ch 6, 100 %, 20 dBm, `6m`, TX on).

```bash
tools/esp32c6_ctl.py send --port /dev/cu.usbmodem101 "ch 11"
tools/esp32c6_ctl.py status
```

---

## The web dashboard

**http://127.0.0.1:8760/** — a browser UI for everything above, across every board
at once. Hot-plug aware: a background scanner picks up boards as they appear, so
plugging one in is all it takes.

### It is always running

The dashboard is installed as a macOS **LaunchAgent**, `com.beesdr.esp32c6-dashboard`:

- `RunAtLoad` — starts at login
- `KeepAlive` — restarted automatically if it exits
- `ProcessType: Background`
- bound to `127.0.0.1:8760` (this Mac only)

There is nothing to launch, and it survives reboots. To check on it:

```bash
dashboard/deploy.sh --status
launchctl print "gui/$(id -u)/com.beesdr.esp32c6-dashboard"
tail -f ~/Library/Logs/beesdr-esp32c6-dashboard.error.log
```

`serve.py` defaults to `--host 0.0.0.0` (reachable from a phone on the same LAN),
but **the installed service overrides it to `127.0.0.1`**. Widen that only
deliberately: the UI has no authentication, and anyone who can reach the port can
key any transmitter.

### Why the service runs from a copy

⚠️ **The service does not run from this folder, and it cannot.** launchd-spawned
processes get no TCC grant for `~/Documents`, so a plist pointing here dies on every
start with a message that reads like a missing file but is not:

```
can't open file '.../dashboard/serve.py': [Errno 1] Operation not permitted
```

`~/Library/Application Support/` is outside TCC, so the service runs from a copy at
`~/Library/Application Support/BEESDR/ESP32C6-dashboard/`.

**This repo is the source of truth; that copy is a deployment artifact.** Editing
`serve.py` or `index.html` here changes nothing until you deploy:

```bash
dashboard/deploy.sh            # sync to the runtime copy + restart the service
dashboard/deploy.sh --status   # loaded? answering?
```

The alternative — granting Full Disk Access to `/usr/bin/python3` — would hand that
privilege to every Python script on the machine. Not worth it for this.

### Iterating without the service

```bash
dashboard/dashboard.sh                                # foreground, 0.0.0.0:8760
dashboard/dashboard.sh --host 127.0.0.1 --port 9000   # somewhere else
```

Stop the service first, or use a different port — **one process per serial port**.
The same applies to `esp32c6_ctl.py`: it cannot drive a board the running dashboard
is holding.

### HTTP API

The UI is a thin client over this; it is equally usable from `curl` or a script.

| Method | Path | Body | Returns |
|---|---|---|---|
| `GET` | `/` | — | the UI |
| `GET` | `/api/devices` | — | `{devices: [...], time}` |
| `POST` | `/api/set` | `{port, ...fields}` | `{device}` |
| `POST` | `/api/all` | `{...fields}` | `{applied: [ports], devices}` |
| `POST` | `/api/raw` | `{port, cmd}` | `{resp, device}` |

Settable fields, applied in this order and silently skipped when out of range:
`ch` (1–14), `duty` (0–100), `pwr` (2–20), `rate` (one of the names above), `tx`
(`true` / `"on"` / `1` → on, anything else → off).

Each device reports `port`, `mac`, `ch`, `freq`, `duty`, `pwr`, `rate`, `tx`,
`frames_ok`, `is_ours` (does it speak this firmware's protocol) and `open_error`.

```bash
curl -s localhost:8760/api/devices | python3 -m json.tool

curl -s -X POST localhost:8760/api/all \
     -H 'Content-Type: application/json' \
     -d '{"ch":6,"duty":20,"pwr":20,"tx":true}'
```

---

## Why beacons, not data frames

`esp_wifi_80211_tx()` rejects data frames whose addressing doesn't match the
interface, but passes arbitrary **management** frames — so beacons are the reliable
injection primitive across IDF versions.

Each beacon carries the current channel in both its SSID (`BEESDR-TX-chNN`) and its
DS-parameter IE, plus a BSSID equal to the board's own (Espressif-OUI) MAC. A
sniffer, or the SDR operator, can therefore read the channel straight off the air
and tell boards apart.

The beacons are standards-conformant so **any scanner lists each board as an open
access point**: SSID, Supported Rates, DS Parameter Set, **TIM** (mandatory in
beacons), ERP, Extended Supported Rates, a live **TSF timestamp**, plus HT and HE
capability/operation IEs when the PHY is n / ax (a scanner takes the Wi-Fi 4/6
label from these IEs, not from the PHY the frame arrived on). The HE BSS colour
is derived from the MAC, so co-located boards differ.

**Verified 2026-11-09** with a fresh CoreWLAN scan on this Mac: the board shows as an
open network on its channel, labelled `b/g`, `b/g/n` or `ax` to match the selected
`rate`, and disappears with `tx off`.

To a receiver this reads as real WiFi: bursty, 20 MHz OFDM-shaped energy on a known
channel.

---

## Notes / gotchas

- **Channels 12–14 are region-restricted** for normal WiFi. The firmware sets a
  manual country policy spanning 1–14 so injection isn't silently clamped. This is
  deliberate lab injection and it is your responsibility to keep it to shielded or
  otherwise authorized setups. Stick to 1–11 unless you specifically need the top
  of the band.
- **PHY rate shapes the spectrum.** `6m` and above are 20 MHz OFDM; `1m/2m/5m/11m`
  are narrower DSSS/CCK. Default is `6m`.
- **`duty=100` is ~99% on-air**, not 100 — one 1 ms scheduler yield per 100 ms
  window keeps the watchdog and idle task alive. Irrelevant for RF purposes.
- **One port = one board, one process.** The dashboard service holds every port it
  finds; stop it before flashing or before using `esp32c6_ctl.py`.
- **The dashboard has no authentication.** Keep it on `127.0.0.1` unless you have a
  reason not to.
- **A charge-only USB cable** lights the LED and enumerates nothing.

---

## Status

Firmware and tooling written 2026-08-31; split out into this standalone project on
2026-09-06.

**Builds clean** against the shared ESP-IDF v5.5.5 (verified 2026-09-06):
`beesdr_wifi_tx.bin`, 0xdb610 bytes, 14% free in the app partition. The dashboard
runs as a login service and is serving on 8760. Flashing to boards and measuring
the emitted spectrum is the remaining step to exercise on hardware.
