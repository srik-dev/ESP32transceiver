# ESP32-Tx — ESP32 WiFi / Bluetooth LE transmitters

Firmware and host tooling that turn one or more **ESP32-C6 or ESP32-S3** dev boards into
controllable 2.4 GHz **WiFi or Bluetooth LE** emitters, each parked on a channel you choose, with a
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
| **Service label** | `com.beesdr.esp32tx-dashboard` |
| **Runs from** | `~/Library/Application Support/BEESDR/ESP32Tx-dashboard/` (**not** this folder — see [Why the service runs from a copy](#why-the-service-runs-from-a-copy)) |
| **Logs** | `~/Library/Logs/beesdr-esp32tx-dashboard{,.error}.log` |
| **Runtime deps** | System `python3` only — stdlib, no venv, no pyserial |
| **Toolchain** | Shared ESP-IDF **v5.5.5** at `~/esp/esp-idf` |
| **Targets** | `esp32c6`, `esp32s3` (one source tree; `flash.sh` picks per board) |

Plug a board into USB and it appears in the dashboard within a couple of seconds —
there is nothing to start.

```bash
open http://127.0.0.1:8760/          # the UI
dashboard/deploy.sh --status         # is the service loaded and answering?
```

---

## Scope

**What this is.** A lab-bench RF signal generator built out of commodity ESP32-C6 / ESP32-S3
boards, plus everything needed to drive it: firmware, a serial control tool, and an
always-on web UI. One firmware image serves every board; boards are differentiated
at runtime by the channel you assign, so a bag of identical boards becomes a
multi-channel emitter set.

**What it is deliberately not:**

- **Not a WiFi client or AP.** The boards never associate. They inject raw 802.11
  beacon frames and nothing else.
- **Not a receiver.** Transmit-only: there is no capture, demodulation or
  sniffing path.
- **Not a BLE peripheral.** BLE mode only broadcasts (advertising or a DTM test
  carrier); nothing can connect to a board. See [Bluetooth LE mode](#bluetooth-le-mode).
- **Not WiFi and BLE at once.** One radio: a board is in one mode or the other.
- **Not remotely controlled over WiFi.** Control is **USB serial only**, by
  necessity — the C6 / S3 is a single-radio, 2.4 GHz-only part and cannot stay
  associated to an access point while injecting on a channel of your choosing. The
  dashboard is a web UI *on the Mac* that bridges HTTP to the USB serial ports; the
  boards themselves are never on the network.
- **Not a general ESP-IDF template.** It is one application for two targets.

**Intended use.** Deliberate, authorized injection on a bench or in a shielded
setup. See [Notes / gotchas](#notes--gotchas) about channels 12–14.

---

## Repository layout

```
ESP32Tx/
├── README.md                  this file
├── .gitignore                 ignores IDF build output (see "Generated" below)
│
├── firmware/                  ESP-IDF project, targets esp32c6 / esp32s3
│   ├── CMakeLists.txt         project file; finds IDF via $ENV{IDF_PATH}
│   ├── sdkconfig.defaults     tracked config — sdkconfig is derived from it
│   ├── sdkconfig.defaults.esp32c6   C6-only overrides (enables BLE DTM)
│   └── main/
│       ├── CMakeLists.txt     component registration
│       ├── idf_component.yml  managed deps (espressif/led_strip)
│       ├── beacon_tx.c        boot, WiFi TX loop, serial command REPL, NVS
│       └── ble_tx.c / .h      Bluetooth LE mode: advertising, Coded PHY, DTM
│
├── tools/                     host-side, run from the repo root
│   ├── idf_env.sh             SOURCE this to activate the shared ESP-IDF
│   ├── setup_idf.sh           one-time IDF install (reference / fallback)
│   ├── flash.sh               build once, flash one or many boards
│   ├── esp32tx_ctl.py         enumerate boards, assign channels (stdlib only)
│   └── ch340_usb.py           libusb driver for CH340K bridges macOS has no driver for
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
| `firmware/build/`, `firmware/build-esp32s3/` | CMake/ninja output per target (~215 MB each), incl. `beesdr_wifi_tx.bin` |
| `firmware/sdkconfig`, `firmware/sdkconfig.esp32s3` | derived from `sdkconfig.defaults*` on first build |
| `firmware/managed_components/` | fetched by the IDF component manager |
| `firmware/dependencies.lock` | resolved component versions |

⚠️ **A build tree is pinned to its absolute path.** CMake records its own source
and binary directory, so *moving this project invalidates `build/`* — the next
`idf.py build` fails on the stale cache rather than on your code. After any move,
delete `firmware/build/` and rebuild.

---

## Hardware

Any ESP32-C6 or ESP32-S3 dev board. Two USB wiring styles exist and the tooling handles both,
but they are worth telling apart because they enumerate differently:

| Path | Enumerates as | Console on |
|---|---|---|
| Native USB-Serial-JTAG (VID `0x303a`) | `/dev/cu.usbmodem*` | the native peripheral |
| USB-UART bridge, e.g. WCH CH343 (VID `0x1a86`) | `/dev/cu.usbmodem*`, `usbserial*`, `wchusbserial*` | UART0 |

The boards in use here are the **CH343 bridge** variety, with the firmware console
on **UART0**. `flash.sh`, `esp32tx_ctl.py` and `serve.py` all glob every pattern
above and filter out the macOS Bluetooth and `debug-console` pseudo-ports, so
either style is picked up automatically.

Whichever the board, one USB cable carries the log, the flash channel and the
interactive REPL on the **same** port — no BOOT/RESET button dance for normal
flashing.

### ESP32-S3 boards

The same firmware also builds for **`esp32s3`** (`tools/flash.sh` reads the chip
type off each port and builds the right target into `firmware/build-esp32s3/`).
Differences from the C6:

- **No 802.11ax.** The S3 has no HE radio, so the `he-mcs*` rates are compiled
  out and answer `ERR unknown rate`; b / g / n all work.
- **Status LED on GPIO48** (DevKitC-1 v1.0 and most clones). v1.1 boards moved it
  to GPIO38 — build with `-DLED_GPIO=38`.
- The dashboard shows each card's chip (`status` reports `chip=`) and offers only
  the standards that chip can transmit.

### Driverless CH340K bridges (`ch340:` ports)

A CH340K bridge (USB `1a86:7522`) gets **no `/dev/cu.*` port on macOS** — Apple's
built-in driver only matches `1a86:7523` and `1a86:55d4`. Rather than requiring
WCH's driver (admin + system-extension approval), `tools/ch340_usb.py` drives the
bridge from userspace through libusb. It is stdlib-only (`ctypes` against
Homebrew's `libusb`), so the dashboard and `esp32tx_ctl.py` use it on the system
`python3`, and `deploy.sh` copies it next to `serve.py`.

Such a board appears everywhere under a topology-based name,
**`ch340:<bus>-<port path>`** (e.g. `ch340:0-1.2`) — stable for a given USB socket,
and accepted wherever a `/dev/cu.*` path is:

```bash
tools/ch340_usb.py list                 # which bridges are attached
tools/flash.sh                          # autodetects them along with /dev ports
tools/esp32tx_ctl.py send --port ch340:0-1.2 "ch 11"
tools/ch340_usb.py reset ch340:0-1.2    # reboot the board and print its boot log
```

Flashing goes through `ch340_usb.py esptool`, which enters the bootloader itself
and hands esptool a pyserial-compatible port. `idf.py monitor` cannot attach to
these ports; `flash.sh --monitor` prints the boot log instead. As with a tty, only
one process can hold a bridge — a second gets "access denied (held by another
process)".

**Identifying boards.** The `/dev` name can shift between plug-ins; the **MAC is
the stable identity**, and the dashboard shows it. Several boards can share one Mac
port through a USB hub.

**Gotcha:** a charge-only cable lights the board's LED but produces no `/dev/cu.*`
entry at all. Use a data cable.

---

## Prerequisites

**ESP-IDF v5.1+, targets esp32c6 / esp32s3 — installed at `~/esp/esp-idf` (currently
v5.5.5)**, with the RISC-V (C6) and Xtensa (S3) toolchains under `~/.espressif`.

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

The **host tools need nothing but the system `python3`** — `esp32tx_ctl.py` and the
dashboard talk to the tty through stdlib `termios`. No venv, no pip, no pyserial.

---

## Build & flash

From `firmware/`, with IDF activated:

```bash
. ../tools/idf_env.sh
idf.py set-target esp32c6        # or esp32s3
idf.py build
idf.py -p /dev/cu.usbmodemXXXX flash monitor
```

Or flash every plugged-in board in one shot, from a cold shell:

```bash
tools/flash.sh                                  # every detected port
tools/flash.sh /dev/cu.usbmodem101              # one specific port
tools/flash.sh --monitor /dev/cu.usbmodem101    # flash, then open the monitor
```

**Free the port first** — one process per serial port, and the always-on
dashboard holds every board. Use the card's **Release port** toggle (or its
**Reflash** button, which does the whole thing from the browser). Any other
program with the port open blocks flashing the same way; `lsof /dev/cu.usbmodemXXXX`
names it.

---

## Assign channels to boards

One image serves every board; the channel you assign after flashing is what
differentiates them. Plug in all the boards, then:

```bash
# What's connected, and each board's current state:
tools/esp32tx_ctl.py list

# Round-robin channels 1, 6, 11 across however many boards are present,
# 100% duty, 20 dBm, and start transmitting:
tools/esp32tx_ctl.py assign --channels 1,6,11 --duty 100 --pwr 20

# Bursty emitter (20% on-air), for exercising a receiver's burst gate:
tools/esp32tx_ctl.py assign --channels 6 --duty 20
```

`assign` walks the detected ports in order and hands out the channel list
round-robin — 5 boards + `--channels 1,6,11` → `1,6,11,1,6`. To pin specific boards
to specific ports, pass `--port` (repeatable) in the order you want.

---

## Runtime control (serial commands)

The firmware runs a line REPL (`beesdr-tx>`) on the console. Type these in
`idf.py monitor`, send them with `esp32tx_ctl.py send`, or use the dashboard:

| Command | Effect |
|---|---|
| `mode <wifi\|ble>` | Radio personality. Saved, then the board **reboots** (~1 s) into it. |
| `ch <1-14>` | Set channel. SSID and DS-parameter IE track it. |
| `duty <0-100>` | On-air percent of a 100 ms window. 100 = continuous, lower = bursty. |
| `pwr <2-20>` | Max TX power, dBm. |
| `rate <name>` | PHY rate — this also picks the 802.11 standard (table below). `6m` default. |
| `tx <on\|off>` | Enable / pause injection. |
| `status` | Print MAC, channel, duty, power, rate, `phy`, `rate_err`, tx state, `led`, frame/error counters. |
| `led <name\|#rrggbb>` | Status-LED colour: `off red green blue yellow cyan magenta white orange`, or any hex colour (dimmed to the same safe level as the names; reported back as `led=#rrggbb`). |

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

### Bluetooth LE mode

The chip has one 2.4 GHz radio, so WiFi and BLE are alternatives, not layers.
`mode ble` stores the choice and restarts the firmware with only the BLE stack
(NimBLE, broadcaster-only) running; `mode wifi` goes back. Time-slicing the two
through coexistence would chop both waveforms, which is the opposite of what a
signal source is for. `tx`, `duty`, `led` and `status` work in both modes, and
BLE settings can be changed while in WiFi mode (they apply on the next switch).

| Command | Effect |
|---|---|
| `ble <adv\|ext\|dtm>` | What to transmit (table below). |
| `phy <1m\|2m\|s8\|s2>` | PHY for `ext` and `dtm`. `adv` is always LE 1M. |
| `advch <all\|37,38,39>` | Advertising channels used by `adv` / `ext` (2402 / 2426 / 2480 MHz). |
| `itvl <20-10240>` | Advertising interval, ms (`adv` / `ext`). |
| `bch <0-39>` | `dtm` RF channel; frequency = 2402 + 2·n MHz. |
| `pat <name>` | `dtm` payload: `prbs9 prbs15 11110000 00001111 10101010 01010101 ones zeros`. |
| `len <0-255>` | `dtm` payload bytes; for `ext`, total advertising-data bytes (padded, max 251). |
| `bpwr <dBm>` | BLE TX power, snapped down to a 3 dB step. C6: −15…+20, S3: −24…+20. |

| `ble` | Standard | On air | Seen by a phone? |
|---|---|---|---|
| `adv` | **Bluetooth 4** legacy advertising | `ADV_NONCONN_IND`, LE 1M, on the chosen advertising channels, once per interval | Yes, as `BEESDR-TX-xxxx` |
| `ext` | **Bluetooth 5** extended advertising | `ADV_EXT_IND` on the advertising channels + `AUX_ADV_IND` on a data channel the controller picks. `1m`: 1M/1M · `2m`: primary 1M, secondary 2M · `s8` / `s2`: **Coded PHY** on both | BT5 scanners only; Coded needs long-range support |
| `dtm` | Direct Test Mode (LE Transmitter Test v2) | Back-to-back test packets on **one fixed RF channel**, LE 1M / 2M / Coded S=8 / Coded S=2. `duty` gates it in 100 ms windows | No — it is not advertising |

Advertising is inherently low duty (a sub-millisecond burst per channel per
interval). **For a steady emitter the SDR can lock onto, use `dtm`.**

`status` adds `mode`, `ble`, `bphy`, `coding`, `bch`, `advch`, `itvl`, `bpwr`,
`pat`, `len`, `ble_state` (`off idle adv dtm error`), `ble_err`, `name`, `bmac`.

- **Coded S=2 vs S=8 while advertising** can only be chosen with the BT 5.4
  "Set Extended Advertising Parameters v2" command. The firmware tries it and falls
  back to the controller's default; `coding=ctrl` in `status` means the controller
  picked (normally S=8). **The ESP32-S3 controller reports `ctrl`** (measured
  2026-11-14); **the ESP32-C6 honours the choice** and reports `s8` / `s2` (measured
  2026-11-16). In `dtm` the coding is always exactly what you ask for.
- **Long data on Coded PHY needs a longer interval.** The controller refuses
  extended advertising whose packet does not fit in the interval
  (`ble_err=adv_data:581`, HCI "Packet Too Long"). On the C6 a full 251-byte payload
  at `itvl 20` fails on `s8` and `s2`; `s8` works from `itvl 50`, `s2` from `itvl 30`,
  and 120 bytes fits at `itvl 20`. The dashboard spells this out next to the PHY selector.
- 2M is never used on the primary advertising channels — the spec does not allow it.
- The C6 controller builds DTM out by default; `firmware/sdkconfig.defaults.esp32c6`
  turns it on.
- WiFi + BLE needs the 1.5 MB app slot (`CONFIG_PARTITION_TABLE_SINGLE_APP_LARGE`).
  NVS keeps its offset, so saved settings survive reflashing from an older build.

```bash
tools/esp32tx_ctl.py mode ble                 # every board; --port to pick one
tools/esp32tx_ctl.py send "ble dtm"
tools/esp32tx_ctl.py send "phy s8"
tools/esp32tx_ctl.py send "bch 19"            # 2440 MHz
```

### Settings persist across power cycles

Every accepted `mode` / `ch` / `duty` / `pwr` / `rate` / `tx` / `led` / BLE command is saved to NVS
flash right away and restored at boot. **A board unplugged from the Mac and powered
from any USB supply comes back up transmitting exactly what it was last set to**,
with no host involved. The dashboard (or `esp32tx_ctl.py`) is only needed to *change*
settings. Erasing flash (`idf.py erase-flash`) returns a board to the defaults
(ch 6, 100 %, 20 dBm, `6m`, TX on).

```bash
tools/esp32tx_ctl.py send --port /dev/cu.usbmodem101 "ch 11"
tools/esp32tx_ctl.py status
```

---

## The web dashboard

**http://127.0.0.1:8760/** — a browser UI for everything above, across every board
at once. Hot-plug aware: a background scanner picks up boards as they appear, so
plugging one in is all it takes.

### It is always running

The dashboard is installed as a macOS **LaunchAgent**, `com.beesdr.esp32tx-dashboard`:

- `RunAtLoad` — starts at login
- `KeepAlive` — restarted automatically if it exits
- `ProcessType: Background`
- bound to `127.0.0.1:8760` (this Mac only)

There is nothing to launch, and it survives reboots. To check on it:

```bash
dashboard/deploy.sh --status
launchctl print "gui/$(id -u)/com.beesdr.esp32tx-dashboard"
tail -f ~/Library/Logs/beesdr-esp32tx-dashboard.error.log
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
`~/Library/Application Support/BEESDR/ESP32Tx-dashboard/`.

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
The same applies to `esp32tx_ctl.py`: it cannot drive a board the running dashboard
is holding.

### HTTP API

The UI is a thin client over this; it is equally usable from `curl` or a script.

| Method | Path | Body | Returns |
|---|---|---|---|
| `GET` | `/` | — | the UI |
| `GET` | `/api/devices` | — | `{devices: [...], firmware: {target: build time}, time}` |
| `POST` | `/api/set` | `{port, ...fields}` | `{device}` |
| `POST` | `/api/all` | `{...fields}` | `{applied: [ports], devices}` |
| `POST` | `/api/raw` | `{port, cmd}` | `{resp, device}` |
| `POST` | `/api/release` | `{port, released}` | `{devices}` |
| `POST` | `/api/flash` | `{port}` | `{devices}` (progress in each device's `flash`) |

Settable fields, applied in this order and silently skipped when out of range:
`ch` (1–14), `duty` (0–100), `pwr` (2–20), `rate` (one of the names above), `tx`
(`true` / `"on"` / `1` → on, anything else → off), `led` (`#rrggbb`, `/api/set` only).
A value of the wrong type, a bad `mode` or a bad `led` is a `400` with `{error}`;
an unknown or released port is a `404`.

`mode` (`"wifi"` / `"ble"`) is applied first and waits for the board to reboot. BLE
fields: `ble`, `bphy`, `bch`, `advch` (list, e.g. `[37,39]`), `itvl`, `bpwr`, `pat`,
`len` — same values as the serial commands. In the UI each card has a
**WiFi | Bluetooth LE** switch under its header; **All WiFi** / **All BLE** in the
top bar switch every board.

**Release port / Reflash.** Every card has two buttons at the bottom:

- **Release port** is a toggle. The service normally holds every board's serial
  port; releasing makes it let go of *that one board* so `tools/flash.sh`,
  `esp32tx_ctl.py` or `idf.py monitor` can use it, without stopping the service.
  The card dims; **Reconnect** takes the port back. Unplugging the board clears it.
- **Reflash** (click twice — the first click arms it) releases the port, writes the
  firmware with esptool, and reconnects. Progress and the result show next to the
  button until the board is unplugged; settings in NVS are kept. It works on
  "Unknown serial device" cards too
  (the chip type is read from the ROM bootloader), so a brand-new board can be
  flashed from the browser.

Reflash writes **prebuilt** images and never compiles. `deploy.sh` copies whatever
is in `firmware/build*/` into the service's runtime folder, so after changing the
firmware: build (`idf.py build` or `tools/flash.sh`), then `dashboard/deploy.sh`.
The button's tooltip shows the build time of the image it will write. esptool comes
from the shared IDF python env under `~/.espressif` (override with `ESPTOOL_PYTHON`).

**Board colours.** Click the round swatch at the top-left of a card to pick that
board's LED colour. The choice is stored on the board (NVS) *and* in
`colors.json` next to the running `serve.py` (keyed by MAC; `deploy.sh` never
overwrites it), and re-applied whenever a board reports a different colour — e.g.
after a full-erase reflash. Boards with no pick fall back to the `LED_COLORS`
table in `serve.py`.

Each device reports `port`, `mac`, `ch`, `freq`, `duty`, `pwr`, `rate`, `tx`,
`frames_ok`, `is_ours` (does it speak this firmware's protocol), `probing` (just
opened, first status reply still pending) and `open_error`.

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
  finds; use the card's **Release port** toggle (or stop the service) before
  flashing or using `esp32tx_ctl.py` on that board.
- **The dashboard has no authentication.** Keep it on `127.0.0.1` unless you have a
  reason not to.
- **A charge-only USB cable** lights the LED and enumerates nothing.

---

## Status

Firmware and tooling written 2026-08-31; split out into this standalone project on
2026-09-06.

**BLE mode added 2026-11-14.** Both targets build (C6 0x126f20, S3 0xebe10 bytes).
Exercised on an ESP32-S3: every `ble` / `phy` / channel / power / pattern
combination is accepted by the controller with `ble_err=0`, and legacy advertising
was received by name on the Mac's own Bluetooth. Extended advertising and DTM have
not been observed on air yet (macOS does not report non-connectable extended
advertising).

**Exercised on an ESP32-C6 2026-11-16**, through the dashboard API: all 13 channel
buttons, duty, power, every b / g / n / ax rate (`rate_err=0` throughout, ~3900
beacons/s at `36m`), LED colour, both mode switches, every BLE kind / PHY / channel /
pattern / length / power setting, Release port, Reconnect and Reflash. Still not
observed on air with an SDR: extended advertising and DTM.

**Builds clean** against the shared ESP-IDF v5.5.5. The dashboard runs as a login
service and is serving on 8760.
