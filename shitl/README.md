# ESP32-SHITL-Tx — timed 2.4 GHz transmitters, driven over WiFi

The network-controlled sibling of [ESP32-Tx](../README.md), kept in the same repo
under `shitl/`. Same boards (ESP32-C6, ESP32-S3 untested), same waveforms, same
command names — but the board sits on a WiFi network waiting for commands instead
of on a USB cable, and it transmits **for a set time** instead of until told to stop.

It is a separate firmware and a separate web app: a board runs one firmware or the
other, and the two dashboards (8760 and 8761) are started and stopped
independently and can run side by side. What the two share is the ESP-IDF
activator (`../tools/idf_env.sh`) and the waveform definitions, which this firmware
copies from `../firmware/main/`.

**Every command below is relative to this `shitl/` folder.**

| | ESP32Tx | ESP32-SHITL-Tx |
|---|---|---|
| Control | USB serial | WiFi (HTTP), USB serial as a fallback |
| Transmission | on until `tx off`, survives power cycles | `run <seconds>`; never survives a power cycle |
| WiFi ↔ BLE | `mode` reboots the board | `mode` just selects what the next job sends |
| Firmware update | USB | over the air, with automatic rollback |
| Dashboard | port 8760, holds the serial ports | port 8761, holds no serial port |

## How a job works

There is one 2.4 GHz radio. It cannot stay associated to an access point and
also inject on another channel or hold a BLE test carrier, so a job takes the
radio outright:

1. `run 10` is acknowledged while the board is still online.
2. 0.3 s later (or after the delay you asked for) the board **leaves the network**,
3. transmits the configured waveform for exactly 10 s (measured: 5001 ms for a 5 s job),
4. rejoins the network (about 3–6 s) and reports `last=done`, the real duration and
   the number of frames sent.

While it transmits it cannot be reached over the air. To abort early: `stop` on the
USB serial console, or power-cycle it — a board always boots idle and online.
A job that has not started yet (`delay`) can be cancelled over the air.

## First-time setup of a board (USB, once)

```bash
tools/flash.sh /dev/cu.usbmodemXXXX                               # build + flash
tools/shitltx_ctl.py --port /dev/cu.usbmodemXXXX wifi <ssid> '<password>'
tools/shitltx_ctl.py list                                          # shows net=online and its ip
```

The credentials are stored in the board's NVS, not in this repo. They survive
reflashing and OTA updates. The computer running the dashboard must be on the same
network.

## Dashboard

```bash
dashboard/dashboard.sh start      # http://127.0.0.1:8761/
dashboard/dashboard.sh stop
```

Boards appear by themselves (each broadcasts its status on UDP 8762 once a second;
known addresses are remembered in `dashboard/devices.json` and polled over HTTP).
If broadcasts are blocked on the network, add a board by IP in the header.

Per board: WiFi / BLE selection with every ESP32Tx control, **Transmit for N s**
with an optional start delay, a countdown while the board is away, the result of
the last job, and **Update firmware**, which pushes the last build in
`firmware/build/` (it never compiles — run `tools/flash.sh --build` first).

Scripting the dashboard:

```bash
curl -s localhost:8761/api/devices
curl -s -X POST localhost:8761/api/set -H 'Content-Type: application/json' \
     -d '{"mac":"ac:eb:e6:..","mode":"wifi","ch":6,"rate":"mcs3","pwr":20,"duty":100,"secs":30}'
curl -s -X POST localhost:8761/api/set -H 'Content-Type: application/json' \
     -d '{"mac":"ac:eb:e6:..","mode":"ble","ble":"dtm","bch":19,"bphy":"s8","secs":20,"delay":5}'
curl -s -X POST localhost:8761/api/stop -H 'Content-Type: application/json' -d '{"mac":".."}'
curl -s -X POST localhost:8761/api/ota  -H 'Content-Type: application/json' -d '{"mac":".."}'
```

`/api/set` takes any subset of `mode ch rate pwr duty ble bphy bch advch itvl bpwr
pat len led`; with `secs` (and optional `delay`) it also starts the job. Settings
and the `run` go to the board as one request, and the board stops at the first
error, so a rejected setting never starts a transmission.

## Without the dashboard

```bash
tools/shitltx_ctl.py find                                   # boards heard on the network
tools/shitltx_ctl.py --host IP status
tools/shitltx_ctl.py --host IP send "mode wifi" "ch 6" "rate 36m"
tools/shitltx_ctl.py --host IP run 30 --delay 5 --wait      # prints the result when it is back
tools/shitltx_ctl.py --host IP ota                          # push firmware/build/shitl_tx.bin
```

Or talk to the board directly — it is plain HTTP on port 80:

```bash
curl http://IP/status
printf 'mode wifi\nch 11\nrate he-mcs7\nrun 15\n' | curl --data-binary @- http://IP/cmd
curl --data-binary @firmware/build/shitl_tx.bin http://IP/ota
```

## Commands

Identical to ESP32Tx except where noted. All are accepted over HTTP and serial;
settings are saved on the board.

| Command | |
|---|---|
| `mode <wifi\|ble>` | what the next job transmits — **no reboot** |
| `ch <1-14>` · `rate <name>` · `pwr <2-20>` · `duty <0-100>` | WiFi beacons, as ESP32Tx. Rates: `1m 2m 5m 11m`, `6m…54m`, `mcs0…mcs7`, `he-mcs0…he-mcs9` (C6) |
| `ble <adv\|ext\|dtm>` · `phy` · `bch` · `advch` · `itvl` · `bpwr` · `pat` · `len` | BLE, as ESP32Tx |
| `led <name\|#rrggbb>` | status LED colour |
| `run <secs> [delay]` | **new** — transmit for 0.1–3600 s, after an optional 0–3600 s delay |
| `stop` | **new** — cancel a pending job (over the air) or abort a running one (serial) |
| `wifi <ssid> [password]` · `wifi clear` | **new** — control network |
| `status` | one `STATUS` line; adds `ver part net ap ip rssi apch job job_left_ms last last_ms last_frames` |

There is no `tx on|off`: nothing is on air outside a job.

## Firmware update over the air

The flash holds two app slots (`firmware/partitions.csv`). `POST /ota` writes the
inactive slot, verifies the image (checksum, hash, chip type) and reboots into it.
The new image is confirmed only once it gets an address on the control network;
if it has not within 90 s the board reboots into the previous image. Bump
`PROJECT_VER` in `firmware/CMakeLists.txt` so `ver=` shows which build is running.

USB flashing is needed once per board (bootloader + partition table) and to recover
a board whose network no longer exists (`wifi <ssid> <password>` over serial).

## Things to know

- **No authentication.** Anyone on the control network can key the transmitter or
  replace the firmware. The dashboard itself binds to `127.0.0.1`; the boards do not
  have that option. Use a bench network.
- **TX is real RF**, up to 20 dBm. Channels 12–14 are region-restricted.
- Frame rate depends on the PHY rate and on how busy the channel is: measured
  ~3600 frames/s at `54m` and ~850 /s at `1m` on channel 6.
- The reply to `run` can be lost on a weak link (the job still runs); the
  dashboard then shows an error but the board's `last=` field tells the truth.
- BLE jobs report `ble_err=0` and the real duration, but there is no frame counter
  for BLE.
- A moved repo breaks `firmware/build/` (CMake cache) — delete it.

## Layout

```
firmware/main/shitl_tx.c   settings, commands, the job timer, WiFi injection
firmware/main/net.c        station link, HTTP API, OTA, UDP announce
firmware/main/ble_tx.c     BLE advertising / DTM
tools/flash.sh             build / USB flash        tools/shitltx_ctl.py   CLI (serial + OTA)
dashboard/                 serve.py + index.html, port 8761
../tools/idf_env.sh        shared with ESP32-Tx: activates the one ESP-IDF install
```
