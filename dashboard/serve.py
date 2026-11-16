#!/usr/bin/env python3
"""
serve.py — local web dashboard to control BEESDR ESP32-Tx WiFi / BLE transmitters (ESP32-C6 / ESP32-S3).

Runs a small HTTP server on this Mac that owns each connected board's USB
serial port and drives the firmware's console commands (mode / ch / duty / pwr /
rate / tx / status, plus ble / phy / bch / advch / itvl / bpwr / pat / len in BLE
mode). Open the printed URL in a browser to flip a board between WiFi and BLE,
pick its channel, PHY, duty cycle and power, and start or stop transmitting. Boards are
hot-plug: plug one into a USB hub and its card appears within ~1 s; unplug it and
it drops off.

Pure Python standard library — no pyserial, no Flask, no pip. The boards talk over
a USB-CDC / USB-UART bridge where the baud rate is irrelevant, so we open
/dev/cu.* directly and put it in raw mode with stdlib termios.

Why USB and not WiFi: the ESP32-C6/S3 is a single-radio, 2.4 GHz-only chip. It
cannot join a 5 GHz network, and even on 2.4 GHz it cannot stay associated to an
AP while injecting on a chosen channel (injection retunes the radio off the AP's
channel). USB is the correct control path; put several boards on one USB hub to
beat a shortage of Mac ports. The dashboard can be reached wirelessly from a
phone/laptop by binding to the LAN (default) — the Mac bridges to the USB boards.

Usage:
    python3 serve.py                 # bind 0.0.0.0:8760, reachable on the LAN
    python3 serve.py --host 127.0.0.1  # localhost only
    python3 serve.py --port 9000

Serial-port contention: only one process may hold a port. Each card has a
"Release port" toggle that makes this server let go of that one board, so
tools/esp32tx_ctl.py, tools/flash.sh or idf.py monitor can use it; toggle it back
to reconnect. "Reflash" on a card releases the port, writes the prebuilt firmware
with esptool, and reconnects.
"""
import argparse
import glob
import json
import os
import re
import socket
import subprocess
import sys
import termios
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HERE = os.path.dirname(os.path.abspath(__file__))

# Userspace driver for CH340 bridges macOS has no kernel driver for (no /dev/cu.*
# entry). Sits next to serve.py in the deployed copy, in ../tools in the repo.
# Optional: without it (or without libusb) those boards are simply not listed.
sys.path[:0] = [HERE, os.path.join(HERE, "..", "tools")]
try:
    import ch340_usb
except ImportError:
    ch340_usb = None

# Default board colour by MAC — drives the on-board WS2812 and the UI tag until
# a colour is picked in the dashboard. Re-applied whenever a board reports a
# different one (e.g. after a full-erase reflash). Add rows as boards are
# introduced.
LED_COLORS = {
    "98:a3:16:98:12:14": "red",
    "f0:f5:bd:0e:df:f8": "green",
    "ac:eb:e6:c1:42:cc": "blue",
    "ac:eb:e6:da:78:44": "yellow",
    "ac:eb:e6:d9:a3:ac": "magenta",
}

# Colours picked in the dashboard, by MAC, as "#rrggbb". These override
# LED_COLORS and live next to serve.py (the deployed runtime dir, which deploy.sh
# never overwrites), so a choice survives server restarts and redeploys. The board
# also keeps it in NVS; this copy re-applies it if a reflash erases the board.
COLORS_FILE = os.path.join(HERE, "colors.json")
_colors_lock = threading.Lock()

# Firmware's named colours -> display hex (the UI colour picker needs hex).
NAMED_HEX = {
    "off": "#000000", "red": "#ff0000", "green": "#00ff00", "blue": "#0000ff",
    "yellow": "#ffff00", "cyan": "#00ffff", "magenta": "#ff00ff",
    "white": "#ffffff", "orange": "#ff4000",
}
HEX_RE = re.compile(r"^#[0-9a-f]{6}$")


def _load_colors():
    try:
        with open(COLORS_FILE) as f:
            data = json.load(f)
        return {k.lower(): v.lower() for k, v in data.items()
                if isinstance(v, str) and HEX_RE.match(v.lower())}
    except (OSError, ValueError, AttributeError):
        return {}


USER_COLORS = _load_colors()


def save_user_color(mac, hexcol):
    with _colors_lock:
        USER_COLORS[mac] = hexcol
        tmp = COLORS_FILE + ".tmp"
        with open(tmp, "w") as f:
            json.dump(USER_COLORS, f, indent=2, sort_keys=True)
        os.replace(tmp, COLORS_FILE)


def desired_led(mac):
    """Colour the board should show: dashboard pick, else the built-in tag."""
    return USER_COLORS.get(mac) or LED_COLORS.get(mac)


def to_hex(led):
    """Firmware led= value ('red', '#12ab34', 'rgb', 'off') -> '#rrggbb' or None."""
    if not led:
        return None
    led = led.lower()
    if HEX_RE.match(led):
        return led
    return NAMED_HEX.get(led)


# 2.4 GHz channel -> centre frequency (MHz)
def chan_freq(ch):
    return 2484 if ch == 14 else 2407 + ch * 5


def find_ports():
    pats = ["/dev/cu.usbmodem*", "/dev/cu.usbserial*", "/dev/cu.wchusbserial*",
            "/dev/cu.SLAB_USBtoUART*"]
    ports = []
    for p in pats:
        ports.extend(glob.glob(p))
    if ch340_usb:
        ports.extend(ch340_usb.list_ports())      # "ch340:<bus>-<port path>"
    return sorted(set(p for p in ports
                      if "Bluetooth" not in p and "debug-console" not in p))


def parse_status(line):
    d = {}
    for tok in line.split():
        if "=" in tok:
            k, v = tok.split("=", 1)
            d[k] = v
    return d


class Device:
    """One connected board: persistent raw serial fd + a poller thread that keeps
    a cached status. All serial I/O is serialised by self.lock."""

    def __init__(self, port):
        self.port = port
        self.fd = None
        self.usb = None             # ch340_usb.CH340 when port is "ch340:..."
        self.lock = threading.Lock()
        self.open_error = None
        self.is_ours = False
        self.status = {}
        self.raw = ""
        self.last_seen = 0.0
        self.fps = 0.0
        self._last_ok = None
        self._last_ok_t = None
        self._led_fallback = None   # hex sent via `led rgb` to old firmware
        self._misses = 0            # consecutive status polls with no reply
        self._stop = threading.Event()
        self._thread = None

    # ---- lifecycle ----
    def open(self):
        if ch340_usb and ch340_usb.is_port(self.port):
            usb = ch340_usb.CH340(self.port)
            usb.open()
            self.usb = usb
            return
        fd = os.open(self.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            a = termios.tcgetattr(fd)
            a[0] = 0            # iflag: raw
            a[1] = 0            # oflag: raw
            a[3] = 0            # lflag: no echo/canon/signals
            # cflag: enable receiver, ignore modem lines; clear HUPCL so closing
            # the port does not drop DTR and reset the board.
            a[2] = (a[2] | termios.CLOCAL | termios.CREAD) & ~termios.CRTSCTS
            a[2] &= ~termios.HUPCL
            try:
                a[4] = a[5] = termios.B115200
            except AttributeError:
                pass
            a[6][termios.VMIN] = 0
            a[6][termios.VTIME] = 0
            termios.tcsetattr(fd, termios.TCSANOW, a)
            termios.tcflush(fd, termios.TCIOFLUSH)
        except termios.error as e:
            self.open_error = "termios: %s" % e
        self.fd = fd

    def close(self):
        self._stop.set()
        with self.lock:
            if self.usb is not None:
                self.usb.close()
                self.usb = None
        if self.fd is not None:
            try:
                os.close(self.fd)
            except OSError:
                pass
            self.fd = None

    def start(self):
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        # small delay so a board that reset on open finishes booting
        self._stop.wait(0.4)
        while not self._stop.is_set():
            try:
                self.refresh()
            except OSError:
                pass
            self._stop.wait(1.0)

    # ---- serial I/O (all under lock) ----
    def _read_for(self, seconds):
        end = time.time() + seconds
        buf = bytearray()
        while time.time() < end:
            try:
                if self.usb is not None:
                    c = self.usb.read(4096, 20)
                    if c:
                        buf.extend(c)
                    continue
                c = os.read(self.fd, 4096)
            except BlockingIOError:
                c = b""
            except OSError:
                break
            if c:
                buf.extend(c)
            else:
                time.sleep(0.02)
        return buf.decode("utf-8", "replace")

    def exchange(self, cmd, wait=0.4):
        with self.lock:
            if self.usb is not None:
                try:
                    self.usb.flush_input()
                    self.usb.write((cmd + "\r\n").encode())
                except OSError:
                    return ""
                return self._read_for(wait)
            if self.fd is None:
                return ""
            try:
                termios.tcflush(self.fd, termios.TCIFLUSH)
            except (termios.error, OSError):
                pass
            try:
                os.write(self.fd, (cmd + "\r\n").encode())
            except OSError:
                return ""
            return self._read_for(wait)

    def refresh(self):
        resp = self.exchange("status", 0.5)
        line = next((l for l in resp.splitlines() if "STATUS " in l), None)
        now = time.time()
        if line:
            self.is_ours = True
            self._misses = 0
            self.raw = line.strip()
            self.status = parse_status(line)
            self.last_seen = now
            try:
                ok = int(self.status.get("frames_ok", "0"))
                if self._last_ok is not None and now > (self._last_ok_t or 0):
                    dt = now - self._last_ok_t
                    # the counter restarts from 0 when a mode switch reboots the board
                    self.fps = max(0.0, (ok - self._last_ok) / dt)
                self._last_ok, self._last_ok_t = ok, now
            except ValueError:
                pass
            # keep the board's colour tag applied (a reflash wipes it)
            desired = desired_led(self.status.get("mac", ""))
            if (desired and self.status.get("led") != desired
                    and self._led_fallback != desired):
                self.set_led(desired)
        else:
            # A mode switch reboots the board (~1 s of boot log instead of a
            # STATUS line); don't drop its card over a couple of missed polls.
            self._misses += 1
            if self._misses >= 4:
                self.is_ours = False

    def set_led(self, colour):
        """Send a named or '#rrggbb' colour. Firmware older than the hex `led`
        command answers ERR; fall back to `led rgb` at the same dimmed level and
        remember it, since that firmware can only report led=rgb (otherwise the
        poller would resend it every second)."""
        resp = self.exchange("led %s" % colour, 0.3)
        if "ERR" in resp and colour.startswith("#"):
            v = int(colour[1:], 16)
            r, g, b = ((((v >> s) & 0xff) * 48 + 254) // 255 for s in (16, 8, 0))
            resp = self.exchange("led rgb %d %d %d" % (r, g, b), 0.3)
            self._led_fallback = colour
        else:
            self._led_fallback = None
        return resp

    def as_dict(self):
        s = self.status
        def geti(k, d=0):
            try:
                return int(re.sub(r"[^0-9-]", "", s.get(k, "")))
            except ValueError:
                return d
        return {
            "port": self.port,
            "is_ours": self.is_ours,
            # just opened: no status reply yet, but not enough misses to call
            # it a stranger (the UI shows "Connecting" instead of a warning)
            "probing": (not self.is_ours and self._misses < 4
                        and self._thread is not None),
            "open_error": self.open_error,
            "mac": s.get("mac", ""),
            # firmware older than the S3 port has no chip= and is always a C6
            "chip": s.get("chip", "esp32c6" if s else ""),
            "ch": geti("ch"),
            "freq": chan_freq(geti("ch")) if s.get("ch") else None,
            "duty": geti("duty"),
            "pwr": geti("pwr"),
            "rate": s.get("rate", ""),
            "phy": s.get("phy", ""),              # "b" / "g" / "n" / "ax"
            "rate_err": s.get("rate_err", "0"),   # IDF error name if the PHY rejected it
            "tx": s.get("tx", ""),
            "frames_ok": geti("frames_ok"),
            "frames_err": geti("frames_err"),
            "bp": geti("bp"),
            "ssid": s.get("ssid", ""),
            "led": s.get("led", ""),
            # display colour: what the board reports, else what it should show
            # (old firmware can only report led=rgb, which has no hex)
            "color": to_hex(s.get("led", ""))
                     or to_hex(desired_led(s.get("mac", ""))),
            # ---- WiFi / BLE personality. Firmware older than BLE support has
            # no mode= field: it is WiFi-only and the UI hides the switch.
            "mode": s.get("mode", "wifi"),
            "has_ble": "mode" in s,
            "ble": s.get("ble", ""),              # adv | ext | dtm
            "bphy": s.get("bphy", ""),            # 1m | 2m | s8 | s2
            "coding": s.get("coding", "-"),       # coded PHY actually on air: s8|s2|ctrl|-
            "bch": geti("bch"),                   # DTM RF channel 0-39
            "advch": [int(c) for c in s.get("advch", "").split(",") if c.isdigit()],
            "itvl": geti("itvl"),
            "bpwr": geti("bpwr"),
            "bpwr_min": geti("bpwr_min", -15),
            "pat": s.get("pat", ""),
            "len": geti("len"),
            "ble_state": s.get("ble_state", ""),
            "ble_err": s.get("ble_err", "0"),
            "name": s.get("name", ""),
            "bmac": s.get("bmac", ""),
            "fps": round(self.fps, 1),
            "age": round(time.time() - self.last_seen, 1) if self.last_seen else None,
        }


# ------------------------------------------------------------- reflashing ---
# Reflash writes PREBUILT images; it never compiles. They are looked for in
#   <here>/firmware/<target>/         the deployed copy (deploy.sh puts them there)
#   ../firmware/build[-<target>]/     the repo's build trees (foreground runs)
# each holding flash_args plus the three .bin files it names.
TARGETS = ("esp32c6", "esp32s3")


def image_dir(target):
    cands = [os.path.join(HERE, "firmware", target),
             os.path.join(HERE, "..", "firmware",
                          "build" if target == "esp32c6" else "build-" + target)]
    for d in cands:
        if os.path.isfile(os.path.join(d, "flash_args")):
            return d
    return None


def firmware_info():
    """target -> build time of the image a reflash would write (None = missing)."""
    out = {}
    for t in TARGETS:
        d = image_dir(t)
        app = os.path.join(d, "beesdr_wifi_tx.bin") if d else None
        out[t] = os.path.getmtime(app) if app and os.path.isfile(app) else None
    return out


def esptool_python():
    """A python that can `import esptool`: $ESPTOOL_PYTHON, else the shared
    ESP-IDF tools env (the stdlib-only system python this server runs on has none)."""
    env = os.environ.get("ESPTOOL_PYTHON")
    if env:
        return env
    root = os.environ.get("IDF_TOOLS_PATH") or os.path.expanduser("~/.espressif")
    cands = sorted(glob.glob(os.path.join(root, "python_env", "*", "bin", "python")))
    return cands[-1] if cands else None


def esptool_cmd(py, port, args):
    if ch340_usb and ch340_usb.is_port(port):
        return [py, os.path.abspath(ch340_usb.__file__), "esptool", port] + args
    return [py, "-m", "esptool", "-p", port] + args


class Manager:
    def __init__(self):
        self.devices = {}
        self.lock = threading.Lock()
        # Ports this server has let go of: port -> {"why": "released"|"flashing",
        # "last": the card's last state}. scan() leaves them alone.
        self.held = {}
        self.flash = {}     # port -> {"state", "pct", "log", "time", "target"}

    # ---- release / reconnect ----
    def release(self, port, why="released"):
        with self.lock:
            if port in self.held:
                self.held[port]["why"] = why
                return True
            d = self.devices.pop(port, None)
            if d is None:
                return False
            last = d.as_dict()
            self.held[port] = {"why": why, "last": last}
        d.close()
        if d._thread:
            d._thread.join(2.0)     # let an in-flight status poll finish
        return True

    def attach(self, port):
        with self.lock:
            self.held.pop(port, None)
        self.scan()

    # ---- reflash ----
    def start_flash(self, port):
        with self.lock:
            if self.flash.get(port, {}).get("state") == "running":
                raise ValueError("already flashing")
            was_released = self.held.get(port, {}).get("why") == "released"
            known = port in self.devices or port in self.held
            self.flash[port] = {"state": "running", "pct": 0, "log": "", "time": time.time(),
                                "target": ""}
        if not known:
            self.flash.pop(port, None)
            raise ValueError("no such device")
        self.release(port, "flashing")
        threading.Thread(target=self._flash, args=(port, was_released), daemon=True).start()

    def _run(self, port, cmd, cwd=None):
        """Run one esptool command, streaming its output into the flash record."""
        st = self.flash[port]
        try:
            proc = subprocess.Popen(cmd, cwd=cwd, stdout=subprocess.PIPE,
                                    stderr=subprocess.STDOUT)
        except OSError as e:
            st["log"] += "%s\n" % e
            return 1, ""
        out = ""
        while True:
            chunk = os.read(proc.stdout.fileno(), 4096)
            if not chunk:
                break
            out += chunk.decode("utf-8", "replace")
            st["log"] = (st["log"] + chunk.decode("utf-8", "replace"))[-4000:]
            pcts = re.findall(r"\((\d+) %\)", out[-200:])
            if pcts:
                st["pct"] = int(pcts[-1])
        return proc.wait(), out

    def _flash(self, port, was_released):
        st = self.flash[port]
        ok = False
        try:
            py = esptool_python()
            if not py:
                raise RuntimeError("no python with esptool found (looked in "
                                   "~/.espressif/python_env; set ESPTOOL_PYTHON)")
            target = self.held.get(port, {}).get("last", {}).get("chip", "")
            if target not in TARGETS:
                # never ran our firmware (or is silent): ask the ROM bootloader
                rc, out = self._run(port, esptool_cmd(py, port, ["chip_id"]))
                m = re.search(r"ESP32-[A-Z][0-9]+", out)
                target = m.group(0).lower().replace("-", "") if m else ""
            st["target"] = target
            if target not in TARGETS:
                raise RuntimeError("could not identify the chip on %s" % port)
            imgs = image_dir(target)
            if not imgs:
                raise RuntimeError("no firmware image for %s - build it (tools/flash.sh) "
                                   "and run dashboard/deploy.sh" % target)
            rc, _ = self._run(port, esptool_cmd(py, port, [
                "--chip", target, "-b", "460800", "--after", "hard_reset",
                "write_flash", "@flash_args"]), cwd=imgs)
            ok = (rc == 0)
        except Exception as e:      # report every failure on the card
            st["log"] = (st["log"] + "%s\n" % e)[-4000:]
        st["state"] = "ok" if ok else "failed"
        st["time"] = time.time()
        if was_released:
            with self.lock:
                if port in self.held:
                    self.held[port]["why"] = "released"
        else:
            time.sleep(0.5)         # let the board come out of reset
            self.attach(port)

    def scan(self):
        ports = set(find_ports())
        with self.lock:
            for p in list(self.flash):
                # a flash result belongs to the board that was plugged in then
                if p not in ports and self.flash[p]["state"] != "running":
                    del self.flash[p]
            for p, d in list(self.devices.items()):
                # the port could not be opened (busy, permissions): try again
                if d.fd is None and d.usb is None:
                    del self.devices[p]
            for p in list(self.held):
                # an unplugged board forgets its release; mid-flash the port may
                # blink out while the chip resets, so keep that one
                if p not in ports and self.held[p]["why"] != "flashing":
                    del self.held[p]
            ports -= set(self.held)
            for p in ports:
                if p not in self.devices:
                    d = Device(p)
                    try:
                        d.open()
                        d.start()
                    except OSError as e:
                        d.open_error = str(e)
                    self.devices[p] = d
            for p in list(self.devices):
                if p not in ports:
                    self.devices[p].close()
                    del self.devices[p]

    def get(self, port):
        with self.lock:
            return self.devices.get(port)

    def snapshot(self):
        with self.lock:
            devs = list(self.devices.values())
            held = {p: dict(h["last"], released=True, why=h["why"])
                    for p, h in self.held.items()}
        out = [d.as_dict() for d in devs if d.port not in held] + list(held.values())
        for d in out:
            d.setdefault("released", False)
            f = self.flash.get(d["port"])
            if f:
                d["flash"] = {"state": f["state"], "pct": f["pct"], "time": f["time"],
                              "target": f["target"],
                              "log": f["log"][-600:] if f["state"] != "ok" else ""}
        return sorted(out, key=lambda x: x["port"])


MGR = Manager()

VALID_RATES = ({"1m", "2m", "5m", "11m",                                  # 802.11b
                "6m", "9m", "12m", "18m", "24m", "36m", "48m", "54m"}       # 802.11g
               | {"mcs%d" % i for i in range(8)}                           # 802.11n HT20
               | {"he-mcs%d" % i for i in range(10)})                      # 802.11ax HE20


BLE_KINDS = {"adv", "ext", "dtm"}
BLE_PHYS = {"1m", "2m", "s8", "s2"}
BLE_PATTERNS = {"prbs9", "11110000", "10101010", "prbs15", "ones", "zeros",
                "00001111", "01010101"}


def apply_ble(dev, body, out):
    if "ble" in body and str(body["ble"]) in BLE_KINDS:
        out.append(dev.exchange("ble %s" % body["ble"], 0.3))
    if "bphy" in body and str(body["bphy"]) in BLE_PHYS:
        out.append(dev.exchange("phy %s" % body["bphy"], 0.3))
    if "bch" in body and 0 <= int(body["bch"]) <= 39:
        out.append(dev.exchange("bch %d" % int(body["bch"]), 0.3))
    if "advch" in body:
        chans = sorted({int(c) for c in body["advch"]} & {37, 38, 39})
        if chans:
            out.append(dev.exchange("advch %s" % ",".join(map(str, chans)), 0.3))
    if "itvl" in body and 20 <= int(body["itvl"]) <= 10240:
        out.append(dev.exchange("itvl %d" % int(body["itvl"]), 0.3))
    if "bpwr" in body and -24 <= int(body["bpwr"]) <= 20:
        out.append(dev.exchange("bpwr %d" % int(body["bpwr"]), 0.3))
    if "pat" in body and str(body["pat"]) in BLE_PATTERNS:
        out.append(dev.exchange("pat %s" % body["pat"], 0.3))
    if "len" in body and 0 <= int(body["len"]) <= 255:
        out.append(dev.exchange("len %d" % int(body["len"]), 0.3))


def apply_settings(dev, body):
    """Apply provided fields to a device in a sensible order; return responses."""
    out = []
    if "mode" in body:
        mode = str(body["mode"])
        if mode not in ("wifi", "ble"):
            raise ValueError("mode must be wifi or ble")
        if not dev.as_dict()["has_ble"]:
            raise ValueError("firmware has no BLE mode - reflash with tools/flash.sh")
        resp = dev.exchange("mode %s" % mode, 0.3)
        out.append(resp)
        if "rebooting" in resp:
            # the board restarts into the other radio stack; wait for its console
            deadline = time.time() + 5.0
            while time.time() < deadline:
                time.sleep(0.4)
                dev.refresh()
                if dev.status.get("mode") == mode and dev._misses == 0:
                    break
    apply_ble(dev, body, out)
    if "ch" in body:
        ch = int(body["ch"])
        if 1 <= ch <= 14:
            out.append(dev.exchange("ch %d" % ch, 0.3))
    if "duty" in body:
        duty = int(body["duty"])
        if 0 <= duty <= 100:
            out.append(dev.exchange("duty %d" % duty, 0.3))
    if "pwr" in body:
        pwr = int(body["pwr"])
        if 2 <= pwr <= 20:
            out.append(dev.exchange("pwr %d" % pwr, 0.3))
    if "rate" in body:
        rate = str(body["rate"])
        if rate in VALID_RATES:
            out.append(dev.exchange("rate %s" % rate, 0.3))
    if "tx" in body:
        tx = "on" if body["tx"] in (True, "on", "true", 1, "1") else "off"
        out.append(dev.exchange("tx %s" % tx, 0.3))
    if "led" in body:
        led = str(body["led"]).lower()
        mac = dev.status.get("mac", "")
        if not HEX_RE.match(led):
            raise ValueError("led must be #rrggbb")
        if not mac:
            raise ValueError("board has not reported its MAC yet")
        save_user_color(mac, led)
        out.append(dev.set_led(led))
    dev.refresh()
    return out


class Handler(BaseHTTPRequestHandler):
    server_version = "beesdr-esp32tx/1.0"

    def log_message(self, fmt, *args):
        pass  # quiet

    def _send(self, code, body, ctype="application/json"):
        if isinstance(body, (dict, list)):
            body = json.dumps(body).encode()
        elif isinstance(body, str):
            body = body.encode()
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = self.path.split("?", 1)[0]
        if path == "/" or path.startswith("/index"):
            try:
                with open(os.path.join(HERE, "index.html"), "rb") as f:
                    self._send(200, f.read(), "text/html; charset=utf-8")
            except OSError:
                self._send(500, {"error": "index.html missing"})
            return
        if path == "/api/devices":
            self._send(200, {"devices": MGR.snapshot(), "firmware": firmware_info(),
                             "time": time.time()})
            return
        self._send(404, {"error": "not found"})

    def do_POST(self):
        length = int(self.headers.get("Content-Length", "0") or "0")
        raw = self.rfile.read(length) if length else b"{}"
        try:
            body = json.loads(raw or b"{}")
        except ValueError:
            self._send(400, {"error": "bad json"})
            return
        if not isinstance(body, dict):
            self._send(400, {"error": "body must be a JSON object"})
            return

        if self.path == "/api/set":
            port = body.get("port")
            dev = MGR.get(port)
            if not dev:
                self._send(404, {"error": "no such device", "port": port})
                return
            try:
                apply_settings(dev, body)
            except (ValueError, TypeError) as e:      # a bad field value
                self._send(400, {"error": str(e)})
                return
            except OSError as e:
                self._send(500, {"error": str(e)})
                return
            self._send(200, {"device": dev.as_dict()})
            return

        if self.path == "/api/release":
            # {port, released: true|false} - let go of / reconnect one board
            port = body.get("port")
            if body.get("released", True):
                if not MGR.release(port):
                    self._send(404, {"error": "no such device", "port": port})
                    return
            else:
                if MGR.held.get(port, {}).get("why") == "flashing":
                    self._send(409, {"error": "flash in progress"})
                    return
                MGR.attach(port)
            self._send(200, {"devices": MGR.snapshot()})
            return

        if self.path == "/api/flash":
            try:
                MGR.start_flash(body.get("port"))
            except ValueError as e:
                self._send(409, {"error": str(e)})
                return
            self._send(200, {"devices": MGR.snapshot()})
            return

        if self.path == "/api/all":
            # broadcast the same setting to every controllable device
            applied = []
            for d in [x for x in (MGR.get(p["port"]) for p in MGR.snapshot()) if x]:
                if d.is_ours:
                    try:
                        apply_settings(d, body)
                    except (ValueError, TypeError, OSError):
                        continue        # e.g. a WiFi-only board asked for BLE
                    applied.append(d.port)
            self._send(200, {"applied": applied, "devices": MGR.snapshot()})
            return

        if self.path == "/api/raw":
            dev = MGR.get(body.get("port"))
            if not dev:
                self._send(404, {"error": "no such device"})
                return
            resp = dev.exchange(str(body.get("cmd", "")), 0.5)
            dev.refresh()
            self._send(200, {"resp": resp, "device": dev.as_dict()})
            return

        self._send(404, {"error": "not found"})


def lan_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except OSError:
        return None


def main():
    ap = argparse.ArgumentParser(description="BEESDR ESP32-Tx WiFi TX dashboard")
    ap.add_argument("--host", default="0.0.0.0",
                    help="bind address (default 0.0.0.0 = reachable on the LAN; "
                         "use 127.0.0.1 to restrict to this Mac)")
    ap.add_argument("--port", type=int, default=8760, help="HTTP port (default 8760)")
    args = ap.parse_args()

    MGR.scan()
    # background scanner for hot-plug
    def scanner():
        while True:
            try:
                MGR.scan()
            except Exception:
                pass
            time.sleep(1.0)
    threading.Thread(target=scanner, daemon=True).start()

    try:
        httpd = ThreadingHTTPServer((args.host, args.port), Handler)
    except OSError as e:
        print("ERROR: cannot start on %s:%d - %s" % (args.host, args.port, e))
        print("A dashboard is probably already running. Stop it with:")
        print("    pkill -f serve.py")
        print("or run on another port:  python3 serve.py --port 8761")
        sys.exit(1)
    print("BEESDR ESP32-Tx dashboard serving:")
    print("  http://127.0.0.1:%d" % args.port)
    if args.host == "0.0.0.0":
        ip = lan_ip()
        if ip:
            print("  http://%s:%d   (open this from your phone/laptop on the same WiFi)" % (ip, args.port))
        print("  (bound to 0.0.0.0 — anyone on your LAN can control the boards; "
              "use --host 127.0.0.1 to restrict)")
    print("Detected ports: %s" % (", ".join(find_ports()) or "none yet"))
    print("Ctrl-C to stop.")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nstopping...")
        with MGR.lock:
            for d in MGR.devices.values():
                d.close()


if __name__ == "__main__":
    main()
