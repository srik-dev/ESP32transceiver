#!/usr/bin/env python3
"""
serve.py — the ESP32-SHITL-Tx dashboard: a local web UI that drives the boards
over the network. Pure stdlib; runs on the system python3.

    dashboard/dashboard.sh start        # http://127.0.0.1:8761/

It never touches a serial port. Boards are found by the STATUS line they
broadcast once a second on UDP 8762, or added by address; each known board is
then polled over HTTP. Commands go to the board's POST /cmd, firmware to POST /ota.

HTTP API (JSON), for scripting:
    GET  /api/devices
    POST /api/set     {mac, mode, ch, rate, pwr, duty, ble, bphy, bch, advch, itvl,
                       bpwr, pat, len, led, secs, delay}
                      any subset of settings; with `secs` it also starts a job
    POST /api/stop    {mac}             cancel a job that has not started yet
    POST /api/cmd     {mac, lines:[..]} raw command lines
    POST /api/ota     {mac}             push the last build in firmware/build*/
    POST /api/add     {ip}
    POST /api/forget  {mac}
"""
import argparse
import json
import os
import re
import socket
import threading
import time
import urllib.error
import urllib.request
from concurrent.futures import ThreadPoolExecutor
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HERE = os.path.dirname(os.path.abspath(__file__))
FW_DIR = os.path.join(HERE, "..", "firmware")
DEV_FILE = os.path.join(HERE, "devices.json")
ANNOUNCE_PORT = 8762
FRESH_S = 4.0            # a status newer than this means the board is reachable
REJOIN_GRACE_S = 30.0    # after a job, how long "rejoining" is shown before "offline"
OPENER = urllib.request.build_opener(urllib.request.ProxyHandler({}))

# body key -> console command, in the order they are sent
SETTINGS = [("mode", "mode"), ("ch", "ch"), ("rate", "rate"), ("pwr", "pwr"), ("duty", "duty"),
            ("ble", "ble"), ("bphy", "phy"), ("bch", "bch"), ("advch", "advch"), ("itvl", "itvl"),
            ("bpwr", "bpwr"), ("pat", "pat"), ("len", "len"), ("led", "led")]
TOKEN = re.compile(r"^[A-Za-z0-9#,.\-]{1,24}$")
IP_RE = re.compile(r"^\d{1,3}(\.\d{1,3}){3}$")


def parse_status(line):
    line = line.strip()
    if not line.startswith("STATUS "):
        return None
    st = dict(kv.split("=", 1) for kv in line.split()[1:] if "=" in kv)
    return st if st.get("fw") == "shitltx" and "mac" in st else None


def dev_http(ip, path, data=None, timeout=3.0, ctype="text/plain"):
    req = urllib.request.Request("http://%s%s" % (ip, path), data=data)
    if data is not None:
        req.add_header("Content-Type", ctype)
    try:
        with OPENER.open(req, timeout=timeout) as r:
            return r.read().decode("utf-8", "replace")
    except urllib.error.HTTPError as e:
        return e.read().decode("utf-8", "replace")


def firmware_info():
    """The last build per target, as the Update button would push it."""
    out = {}
    for target in ("esp32c6", "esp32s3"):
        d = os.path.join(FW_DIR, "build" if target == "esp32c6" else "build-" + target)
        path = os.path.join(d, "shitl_tx.bin")
        if not os.path.isfile(path):
            continue
        ver = "?"
        try:
            with open(os.path.join(d, "project_description.json")) as fh:
                ver = json.load(fh).get("project_version", "?")
        except (OSError, ValueError):
            pass
        out[target] = {"path": path, "size": os.path.getsize(path),
                       "mtime": os.path.getmtime(path), "ver": ver}
    return out


class Registry:
    def __init__(self):
        self.lock = threading.Lock()
        self.devs = {}          # mac -> dict
        self.pending = set()    # addresses added by hand that have not answered yet
        try:
            with open(DEV_FILE) as fh:
                for mac, ip in json.load(fh).items():
                    if IP_RE.match(str(ip)):
                        self.devs[mac] = self._new(mac, ip)
        except (OSError, ValueError):
            pass

    @staticmethod
    def _new(mac, ip):
        return {"mac": mac, "ip": ip, "st": {}, "seen": 0.0, "job_from": 0.0, "job_until": 0.0,
                "ota": "", "ota_msg": "", "ota_t": 0.0}

    def _save(self):
        try:
            with open(DEV_FILE, "w") as fh:
                json.dump({m: d["ip"] for m, d in self.devs.items()}, fh, indent=1)
        except OSError:
            pass

    def update(self, st, ip):
        now = time.time()
        with self.lock:
            mac = st["mac"]
            d = self.devs.get(mac)
            if d is None:
                d = self.devs[mac] = self._new(mac, ip)
                self._save()
            elif d["ip"] != ip:
                d["ip"] = ip
                self._save()
            self.pending.discard(ip)
            d["st"] = st
            d["seen"] = now
            if d["job_until"] and st.get("job") == "idle" and now > d["job_from"]:
                d["job_from"] = d["job_until"] = 0.0          # the job is over and reported
            if d["ota"] == "rebooting" and now - d["ota_t"] > 2 and int(st.get("up", "999")) < 120:
                d["ota"] = "done"
                d["ota_msg"] = "running v%s from %s" % (st.get("ver"), st.get("part"))

    def targets(self):
        with self.lock:
            return [d["ip"] for d in self.devs.values() if d["ota"] != "uploading"] + list(self.pending)

    def get(self, mac):
        with self.lock:
            return self.devs.get(mac)

    def add(self, ip):
        with self.lock:
            self.pending.add(ip)

    def forget(self, mac):
        with self.lock:
            self.devs.pop(mac, None)
            self._save()

    def snapshot(self):
        now = time.time()
        out = []
        with self.lock:
            for d in sorted(self.devs.values(), key=lambda x: x["mac"]):
                age = now - d["seen"]
                st = d["st"]
                if d["ota"] == "rebooting" and now - d["ota_t"] > 100:
                    d["ota"] = "failed"
                    d["ota_msg"] = "board did not come back; it rolls back to the old image by itself"
                if d["ota"] in ("uploading", "rebooting"):
                    state = "updating"
                elif d["job_until"] and now < d["job_until"]:
                    state = "pending" if now < d["job_from"] else "transmitting"
                elif age < FRESH_S:
                    state = "pending" if st.get("job") == "pending" else "online"
                elif d["job_until"] and now < d["job_until"] + REJOIN_GRACE_S:
                    state = "rejoining"
                else:
                    state = "offline"
                out.append({"mac": d["mac"], "ip": d["ip"], "state": state, "age": round(age, 1),
                            "st": st, "job_from": d["job_from"], "job_until": d["job_until"],
                            "ota": d["ota"], "ota_msg": d["ota_msg"]})
        return {"now": now, "devices": out, "pending": sorted(self.pending),
                "firmware": {t: {k: v for k, v in f.items() if k != "path"}
                             for t, f in firmware_info().items()}}


REG = Registry()


def poll_one(ip):
    try:
        st = parse_status(dev_http(ip, "/status", timeout=1.2))
    except (OSError, ValueError):
        return
    if st:
        REG.update(st, ip)


def poller():
    pool = ThreadPoolExecutor(max_workers=8)
    while True:
        list(pool.map(poll_one, REG.targets()))
        time.sleep(1.0)


def announce_listener():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    if hasattr(socket, "SO_REUSEPORT"):
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    try:
        s.bind(("", ANNOUNCE_PORT))
    except OSError as e:
        print("UDP %d unavailable (%s): boards must be added by address" % (ANNOUNCE_PORT, e))
        return
    while True:
        try:
            data, addr = s.recvfrom(2048)
        except OSError:
            continue
        st = parse_status(data.decode("utf-8", "replace"))
        if st:
            REG.update(st, addr[0])


def build_lines(body):
    lines = []
    for key, cmd in SETTINGS:
        if key in body and body[key] is not None:
            v = str(body[key])
            if isinstance(body[key], bool) or not TOKEN.match(v):
                raise ValueError("bad value for %s" % key)
            lines.append("%s %s" % (cmd, v))
    if body.get("secs") is not None:
        secs = float(body["secs"])
        delay = float(body.get("delay") or 0)
        if not 0.1 <= secs <= 3600:
            raise ValueError("secs must be 0.1-3600")
        if not 0 <= delay <= 3600:
            raise ValueError("delay must be 0-3600")
        lines.append("run %.3f %.3f" % (secs, delay))
    return lines


def send_lines(d, lines):
    """POST command lines to a board. Returns (ok, reply lines)."""
    text = dev_http(d["ip"], "/cmd", ("\n".join(lines) + "\n").encode(), timeout=4.0)
    replies = [ln for ln in text.splitlines() if ln.strip()]
    ok = len(replies) == len(lines) and all(r.startswith(("OK", "STATUS")) for r in replies)
    now = time.time()
    for r in replies:
        m = re.match(r"OK run ms=(\d+) delay_ms=(\d+)", r)
        if m:
            with REG.lock:
                d["job_from"] = now + int(m.group(2)) / 1000.0
                d["job_until"] = d["job_from"] + int(m.group(1)) / 1000.0 + 0.3
        if r.startswith("OK stop"):
            with REG.lock:
                d["job_from"] = d["job_until"] = 0.0
    return ok, replies


def do_ota(d):
    chip = d["st"].get("chip", "esp32c6")
    fw = firmware_info().get(chip)
    if not fw:
        d["ota"], d["ota_msg"] = "failed", "no build for %s (run tools/flash.sh --build)" % chip
        return
    try:
        with open(fw["path"], "rb") as fh:
            blob = fh.read()
        reply = dev_http(d["ip"], "/ota", blob, timeout=180, ctype="application/octet-stream").strip()
    except OSError as e:
        d["ota"], d["ota_msg"] = "failed", "upload failed: %s" % e
        return
    if reply.startswith("OK"):
        d["ota"], d["ota_msg"], d["ota_t"] = "rebooting", "image written, rebooting", time.time()
    else:
        d["ota"], d["ota_msg"] = "failed", reply or "no reply"


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, fmt, *args):
        pass

    def _send(self, code, body, ctype="application/json"):
        data = body if isinstance(body, bytes) else json.dumps(body).encode()
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        path = self.path.split("?")[0]
        if path in ("/", "/index.html"):
            with open(os.path.join(HERE, "index.html"), "rb") as fh:
                self._send(200, fh.read(), "text/html; charset=utf-8")
        elif path == "/api/devices":
            self._send(200, REG.snapshot())
        else:
            self._send(404, {"ok": False, "error": "not found"})

    def do_POST(self):
        path = self.path.split("?")[0]
        try:
            n = int(self.headers.get("Content-Length", "0"))
            body = json.loads(self.rfile.read(n) or b"{}") if n else {}
            if not isinstance(body, dict):
                raise ValueError("body must be a JSON object")
        except ValueError as e:
            return self._send(400, {"ok": False, "error": str(e)})

        if path == "/api/add":
            ip = str(body.get("ip", "")).strip()
            if not IP_RE.match(ip):
                return self._send(400, {"ok": False, "error": "expected an IPv4 address"})
            REG.add(ip)
            threading.Thread(target=poll_one, args=(ip,), daemon=True).start()
            return self._send(200, {"ok": True})

        d = REG.get(str(body.get("mac", "")))
        if d is None:
            return self._send(404, {"ok": False, "error": "unknown device"})
        if path == "/api/forget":
            REG.forget(d["mac"])
            return self._send(200, {"ok": True})
        if path == "/api/ota":
            if d["ota"] in ("uploading", "rebooting"):
                return self._send(409, {"ok": False, "error": "update already in progress"})
            d["ota"], d["ota_msg"] = "uploading", "uploading"
            threading.Thread(target=do_ota, args=(d,), daemon=True).start()
            return self._send(200, {"ok": True})
        try:
            if path == "/api/set":
                lines = build_lines(body)
            elif path == "/api/stop":
                lines = ["stop"]
            elif path == "/api/cmd":
                lines = [str(x) for x in body.get("lines", [])]
                if any("\n" in x or "\r" in x or len(x) > 200 for x in lines):
                    raise ValueError("one command per entry, at most 200 characters")
            else:
                return self._send(404, {"ok": False, "error": "not found"})
            if not lines:
                raise ValueError("nothing to send")
        except (ValueError, TypeError) as e:
            return self._send(400, {"ok": False, "error": str(e)})
        try:
            ok, replies = send_lines(d, lines)
        except OSError as e:
            return self._send(502, {"ok": False, "error": "board unreachable at %s: %s" % (d["ip"], e)})
        err = next((r for r in replies if r.startswith("ERR")), "")
        self._send(200, {"ok": ok, "sent": lines, "reply": replies, "error": err})


def main():
    ap = argparse.ArgumentParser(description="ESP32-SHITL-Tx dashboard")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8761)
    a = ap.parse_args()
    threading.Thread(target=announce_listener, daemon=True).start()
    threading.Thread(target=poller, daemon=True).start()
    srv = ThreadingHTTPServer((a.host, a.port), Handler)
    srv.daemon_threads = True
    print("ESP32-SHITL-Tx dashboard: http://%s:%d/  (listening for boards on UDP %d)"
          % (a.host, a.port, ANNOUNCE_PORT), flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
