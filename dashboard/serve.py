#!/usr/bin/env python3
"""
serve.py — local web dashboard to control BEESDR ESP32-C6 WiFi transmitters.

Runs a small HTTP server on this Mac that owns each connected C6 board's USB
serial port and drives the firmware's console commands (ch / duty / pwr / rate /
tx / status). Open the printed URL in a browser to pick a 2.4 GHz channel per
device, set duty cycle / power / rate, and start or stop injection. Boards are
hot-plug: plug one into a USB hub and its card appears within ~1 s; unplug it and
it drops off.

Pure Python standard library — no pyserial, no Flask, no pip. The C6 talks over
a USB-CDC / USB-UART bridge where the baud rate is irrelevant, so we open
/dev/cu.* directly and put it in raw mode with stdlib termios.

Why USB and not WiFi: the ESP32-C6 is a single-radio, 2.4 GHz-only chip. It
cannot join a 5 GHz network, and even on 2.4 GHz it cannot stay associated to an
AP while injecting on a chosen channel (injection retunes the radio off the AP's
channel). USB is the correct control path; put several boards on one USB hub to
beat a shortage of Mac ports. The dashboard can be reached wirelessly from a
phone/laptop by binding to the LAN (default) — the Mac bridges to the USB boards.

Usage:
    python3 serve.py                 # bind 0.0.0.0:8760, reachable on the LAN
    python3 serve.py --host 127.0.0.1  # localhost only
    python3 serve.py --port 9000

Serial-port contention: only one process may hold a port. Do NOT run
tools/esp32c6_ctl.py at the same time as this server.
"""
import argparse
import glob
import json
import os
import re
import socket
import sys
import termios
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HERE = os.path.dirname(os.path.abspath(__file__))

# Physical board colour by MAC — drives the on-board WS2812 and the UI tag.
# The dashboard re-applies this on every connect, so it survives the
# reset-on-open that wipes the board back to firmware defaults. Add rows as
# boards are introduced.
LED_COLORS = {
    "98:a3:16:98:12:14": "red",
    "f0:f5:bd:0e:df:f8": "green",
    "ac:eb:e6:c1:42:cc": "blue",
}

# 2.4 GHz channel -> centre frequency (MHz)
def chan_freq(ch):
    return 2484 if ch == 14 else 2407 + ch * 5


def find_ports():
    pats = ["/dev/cu.usbmodem*", "/dev/cu.usbserial*", "/dev/cu.wchusbserial*",
            "/dev/cu.SLAB_USBtoUART*"]
    ports = []
    for p in pats:
        ports.extend(glob.glob(p))
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
        self.lock = threading.Lock()
        self.open_error = None
        self.is_ours = False
        self.status = {}
        self.raw = ""
        self.last_seen = 0.0
        self.fps = 0.0
        self._last_ok = None
        self._last_ok_t = None
        self._stop = threading.Event()
        self._thread = None

    # ---- lifecycle ----
    def open(self):
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
            self.raw = line.strip()
            self.status = parse_status(line)
            self.last_seen = now
            try:
                ok = int(self.status.get("frames_ok", "0"))
                if self._last_ok is not None and now > (self._last_ok_t or 0):
                    dt = now - self._last_ok_t
                    self.fps = max(0.0, (ok - self._last_ok) / dt)
                self._last_ok, self._last_ok_t = ok, now
            except ValueError:
                pass
            # keep the board's colour tag applied (reset-on-open wipes it)
            desired = LED_COLORS.get(self.status.get("mac", ""))
            if desired and self.status.get("led") != desired:
                self.exchange("led %s" % desired, 0.2)
        else:
            self.is_ours = False

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
            "open_error": self.open_error,
            "mac": s.get("mac", ""),
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
            "color": LED_COLORS.get(s.get("mac", "")),
            "fps": round(self.fps, 1),
            "age": round(time.time() - self.last_seen, 1) if self.last_seen else None,
        }


class Manager:
    def __init__(self):
        self.devices = {}
        self.lock = threading.Lock()

    def scan(self):
        ports = set(find_ports())
        with self.lock:
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
        return sorted((d.as_dict() for d in devs), key=lambda x: x["port"])


MGR = Manager()

VALID_RATES = ({"1m", "2m", "5m", "11m",                                  # 802.11b
                "6m", "9m", "12m", "18m", "24m", "36m", "48m", "54m"}       # 802.11g
               | {"mcs%d" % i for i in range(8)}                           # 802.11n HT20
               | {"he-mcs%d" % i for i in range(10)})                      # 802.11ax HE20


def apply_settings(dev, body):
    """Apply provided fields to a device in a sensible order; return responses."""
    out = []
    # pause first if channel is changing and tx is being (re)started, so the
    # firmware applies cleanly — but the firmware handles live changes anyway.
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
    dev.refresh()
    return out


class Handler(BaseHTTPRequestHandler):
    server_version = "beesdr-esp32c6/1.0"

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
        if self.path == "/" or self.path.startswith("/index"):
            try:
                with open(os.path.join(HERE, "index.html"), "rb") as f:
                    self._send(200, f.read(), "text/html; charset=utf-8")
            except OSError:
                self._send(500, {"error": "index.html missing"})
            return
        if self.path == "/api/devices":
            self._send(200, {"devices": MGR.snapshot(),
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

        if self.path == "/api/set":
            port = body.get("port")
            dev = MGR.get(port)
            if not dev:
                self._send(404, {"error": "no such device", "port": port})
                return
            try:
                apply_settings(dev, body)
            except (ValueError, OSError) as e:
                self._send(500, {"error": str(e)})
                return
            self._send(200, {"device": dev.as_dict()})
            return

        if self.path == "/api/all":
            # broadcast the same setting to every controllable device
            applied = []
            for d in [x for x in (MGR.get(p["port"]) for p in MGR.snapshot()) if x]:
                if d.is_ours:
                    apply_settings(d, body)
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
    ap = argparse.ArgumentParser(description="BEESDR ESP32-C6 WiFi TX dashboard")
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
    print("BEESDR ESP32-C6 dashboard serving:")
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
