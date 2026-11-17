#!/usr/bin/env python3
"""
shitltx_ctl.py — drive ESP32-SHITL-Tx boards from the command line.

Pure Python stdlib. Two transports, same commands:

  over USB serial (provisioning, or aborting a running job):
    tools/shitltx_ctl.py list                               # ports + STATUS
    tools/shitltx_ctl.py --port PORT wifi SSID 'PASSWORD'   # join a network
    tools/shitltx_ctl.py --port PORT send "ch 11" "rate mcs3"
    tools/shitltx_ctl.py --port PORT send stop

  over the air (the board's address comes from `find`, `list` or the dashboard):
    tools/shitltx_ctl.py find                               # listen for boards
    tools/shitltx_ctl.py --host IP status
    tools/shitltx_ctl.py --host IP send "mode wifi" "ch 6" "rate 36m"
    tools/shitltx_ctl.py --host IP run 10                   # transmit 10 s
    tools/shitltx_ctl.py --host IP run 30 --delay 5 --wait  # ...and wait for the result
    tools/shitltx_ctl.py --host IP send "mode ble" "ble dtm" "bch 19" "run 20"
    tools/shitltx_ctl.py --host IP ota                      # push firmware/build/shitl_tx.bin

`send` over the air runs the lines in order as one request and stops at the
first ERR, so settings and the `run` that uses them go together.
"""
import argparse
import glob
import os
import socket
import sys
import termios
import time
import urllib.error
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
ANNOUNCE_PORT = 8762
OPENER = urllib.request.build_opener(urllib.request.ProxyHandler({}))


# ------------------------------------------------------------------ serial ---

def find_ports():
    ports = []
    for pat in ("/dev/cu.usbmodem*", "/dev/cu.usbserial*", "/dev/cu.wchusbserial*",
                "/dev/cu.SLAB_USBtoUART*"):
        ports.extend(glob.glob(pat))
    return sorted(set(ports))


def open_port(port):
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        iflag, oflag, cflag, lflag, ispeed, ospeed, cc = termios.tcgetattr(fd)
        cflag = (cflag | termios.CLOCAL | termios.CREAD) & ~termios.CRTSCTS
        cc[termios.VMIN] = 0
        cc[termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW,
                          [0, 0, cflag, 0, termios.B115200, termios.B115200, cc])
        termios.tcflush(fd, termios.TCIOFLUSH)
    except termios.error:
        pass
    return fd


def _read_until_reply(fd, timeout):
    """Read until a reply line (OK / ERR / STATUS) is complete, or timeout."""
    end = time.time() + timeout
    buf = bytearray()
    while time.time() < end:
        try:
            chunk = os.read(fd, 4096)
        except BlockingIOError:
            chunk = b""
        except OSError:
            break
        if chunk:
            buf.extend(chunk)
            for ln in buf.decode("utf-8", "replace").split("\n")[:-1]:
                if ln.strip().startswith(("OK", "ERR", "STATUS ")):
                    return ln.strip()
        else:
            time.sleep(0.02)
    return None


def serial_cmd(port, cmd, timeout=1.5):
    fd = open_port(port)
    try:
        os.write(fd, b"\r\n")
        time.sleep(0.08)
        try:
            os.read(fd, 65536)
        except (BlockingIOError, OSError):
            pass
        os.write(fd, (cmd + "\r\n").encode())
        return _read_until_reply(fd, timeout)
    finally:
        os.close(fd)


def quote(arg):
    """Quote one argument for the board's console parser."""
    return '"' + arg.replace("\\", "\\\\").replace('"', '\\"') + '"'


# --------------------------------------------------------------------- OTA ---

def parse_status(line):
    if not line or not line.startswith("STATUS "):
        return None
    return dict(kv.split("=", 1) for kv in line.split()[1:] if "=" in kv)


def http(host, path, data=None, timeout=5, ctype="text/plain"):
    req = urllib.request.Request("http://%s%s" % (host, path), data=data)
    if data is not None:
        req.add_header("Content-Type", ctype)
    try:
        with OPENER.open(req, timeout=timeout) as r:
            return r.read().decode("utf-8", "replace")
    except urllib.error.HTTPError as e:
        return e.read().decode("utf-8", "replace")


def listen(seconds):
    """Collect STATUS announcements for `seconds`; returns {mac: (ip, status)}."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    if hasattr(socket, "SO_REUSEPORT"):
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    s.bind(("", ANNOUNCE_PORT))
    s.settimeout(0.3)
    found = {}
    end = time.time() + seconds
    while time.time() < end:
        try:
            data, addr = s.recvfrom(2048)
        except socket.timeout:
            continue
        st = parse_status(data.decode("utf-8", "replace").strip())
        if st and st.get("fw") == "shitltx":
            found[st["mac"]] = (addr[0], st)
    s.close()
    return found


def summary(st):
    return ("%s  %s v%s (%s)  net=%s ip=%s rssi=%s  mode=%s ch=%s rate=%s  job=%s last=%s/%sms/%s frames"
            % (st.get("mac"), st.get("chip"), st.get("ver"), st.get("part"), st.get("net"),
               st.get("ip"), st.get("rssi"), st.get("mode"), st.get("ch"), st.get("rate"),
               st.get("job"), st.get("last"), st.get("last_ms"), st.get("last_frames")))


def default_image(chip="esp32c6"):
    d = "build" if chip == "esp32c6" else "build-" + chip
    return os.path.join(HERE, "..", "firmware", d, "shitl_tx.bin")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="serial port (USB)")
    ap.add_argument("--host", help="board address (over the air)")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("list", help="serial ports and what answers on them")
    f = sub.add_parser("find", help="listen for boards announcing themselves on the network")
    f.add_argument("--seconds", type=float, default=3.0)
    sub.add_parser("status")
    s = sub.add_parser("send", help="one or more command lines")
    s.add_argument("lines", nargs="+")
    w = sub.add_parser("wifi", help="set the control network (serial)")
    w.add_argument("ssid")
    w.add_argument("password", nargs="?", default="")
    r = sub.add_parser("run", help="transmit the current configuration for N seconds")
    r.add_argument("seconds", type=float)
    r.add_argument("--delay", type=float, default=0.0)
    r.add_argument("--wait", action="store_true", help="wait for the board to come back and print the result")
    o = sub.add_parser("ota", help="push a firmware image over the air")
    o.add_argument("image", nargs="?")
    a = ap.parse_args()

    if a.cmd == "list":
        ports = find_ports()
        if not ports:
            print("no serial ports")
        for p in ports:
            try:
                st = parse_status(serial_cmd(p, "status"))
            except OSError as e:
                print("%s  (cannot open: %s)" % (p, e))
                continue
            if st and st.get("fw") == "shitltx":
                print("%s  %s" % (p, summary(st)))
            else:
                print("%s  (no ESP32-SHITL-Tx status response)" % p)
        return 0

    if a.cmd == "find":
        found = listen(a.seconds)
        if not found:
            print("no boards heard on UDP %d in %.0f s" % (ANNOUNCE_PORT, a.seconds))
            return 1
        for mac, (ip, st) in sorted(found.items()):
            print("%-15s %s" % (ip, summary(st)))
        return 0

    if a.cmd == "wifi":
        lines = ["wifi %s %s" % (quote(a.ssid), quote(a.password)) if a.password
                 else "wifi %s" % quote(a.ssid)]
    elif a.cmd == "status":
        lines = ["status"]
    elif a.cmd == "run":
        lines = ["run %g %g" % (a.seconds, a.delay)]
    elif a.cmd == "send":
        lines = a.lines
    else:
        lines = []

    if a.cmd == "ota":
        if not a.host:
            ap.error("ota needs --host")
        st = parse_status(http(a.host, "/status").strip()) or {}
        image = a.image or default_image(st.get("chip", "esp32c6"))
        with open(image, "rb") as fh:
            blob = fh.read()
        print("pushing %s (%d bytes) to %s, running v%s from %s ..."
              % (os.path.relpath(image), len(blob), a.host, st.get("ver"), st.get("part")))
        reply = http(a.host, "/ota", blob, timeout=180, ctype="application/octet-stream").strip()
        print(reply)
        if not reply.startswith("OK"):
            return 1
        end = time.time() + 60
        time.sleep(3)
        while time.time() < end:
            try:
                new = parse_status(http(a.host, "/status", timeout=2).strip())
                if new:
                    print("back: v%s from %s, up %s s" % (new.get("ver"), new.get("part"), new.get("up")))
                    return 0
            except OSError:
                time.sleep(1)
        print("board did not come back within 60 s (it rolls back by itself after 90 s offline)")
        return 1

    if a.host:
        out = http(a.host, "/cmd", ("\n".join(lines) + "\n").encode()).strip()
        print(out)
        ok = all(ln.startswith(("OK", "STATUS")) for ln in out.splitlines()) and out != ""
        if ok and a.cmd == "run" and a.wait:
            total = a.seconds + max(a.delay, 0.3)
            print("off the network for ~%.1f s ..." % total)
            time.sleep(total)
            end = time.time() + 40
            while time.time() < end:
                try:
                    st = parse_status(http(a.host, "/status", timeout=2).strip())
                    if st and st.get("job") == "idle":
                        print(summary(st))
                        return 0
                except OSError:
                    pass
                time.sleep(1)
            print("board did not rejoin within 40 s")
            return 1
        return 0 if ok else 1

    ports = [a.port] if a.port else find_ports()
    if len(ports) != 1:
        ap.error("pass --port (found: %s) or --host" % (", ".join(ports) or "none"))
    rc = 0
    for ln in lines:
        reply = serial_cmd(ports[0], ln)
        print(reply if reply else "(no reply)")
        if not reply or reply.startswith("ERR"):
            rc = 1
            break
    return rc


if __name__ == "__main__":
    sys.exit(main())
