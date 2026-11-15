#!/usr/bin/env python3
"""
esp32tx_ctl.py — enumerate BEESDR ESP32-Tx boards (ESP32-C6 / ESP32-S3) and tune
each to a WiFi channel over its serial console.

Pure Python stdlib — no pyserial. The C6 exposes the console as a USB-CDC
device, so the baud rate is irrelevant; we open /dev/cu.usbmodem* directly,
put the tty in raw mode with the stdlib `termios`, and exchange the firmware's
line-oriented commands (ch / duty / pwr / rate / tx / status).

Boards on a CH340 bridge macOS has no driver for (CH340K, 1a86:7522) have no
/dev entry; they are reached through tools/ch340_usb.py (libusb) and show up as
ports named ch340:<bus>-<port path>.

Run from the repo root (the project convention), though this tool writes
nothing under Experiments/ so it is path-agnostic.

Examples:
    # See which boards are plugged in
    tools/esp32tx_ctl.py list

    # Round-robin channels 1,6,11 across every detected board, 100% duty, 20 dBm,
    # then turn TX on and print each board's status:
    tools/esp32tx_ctl.py assign --channels 1,6,11 --duty 100 --pwr 20

    # Bursty emitter to exercise the AoA burst gate:
    tools/esp32tx_ctl.py assign --channels 6 --duty 20

    # One-off command to one board:
    tools/esp32tx_ctl.py send --port /dev/cu.usbmodem101 "ch 11"
    tools/esp32tx_ctl.py status

    # Flip every board (or --port ones) to Bluetooth LE and back. The board
    # saves the mode and reboots into it (~1 s):
    tools/esp32tx_ctl.py mode ble
    tools/esp32tx_ctl.py send "ble dtm"      # then: phy s8 / bch 19 / bpwr 9 ...
    tools/esp32tx_ctl.py mode wifi
"""
import argparse
import glob
import os
import sys
import termios
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ch340_usb


def find_ports(explicit=None):
    """Candidate ESP32 console ports. macOS names the native USB-Serial-JTAG
    /dev/cu.usbmodem*; USB-UART bridge kits show as /dev/cu.usbserial* or
    /dev/cu.wchusbserial*. Bluetooth and the debug-console pseudo-ports are
    excluded."""
    if explicit:
        return explicit
    pats = ["/dev/cu.usbmodem*", "/dev/cu.usbserial*", "/dev/cu.wchusbserial*",
            "/dev/cu.SLAB_USBtoUART*"]
    ports = []
    for p in pats:
        ports.extend(glob.glob(p))
    ports.extend(ch340_usb.list_ports())
    ports = [p for p in ports
             if "Bluetooth" not in p and "debug-console" not in p]
    return sorted(set(ports))


def open_port(port):
    """Open a USB-CDC tty in raw mode and return the fd."""
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        attrs = termios.tcgetattr(fd)
        # raw: no canonical mode, no echo, no signal chars, no flow control,
        # no CR/LF translation. Baud is a no-op on USB-CDC but set it anyway.
        iflag, oflag, cflag, lflag, ispeed, ospeed, cc = attrs
        iflag = 0
        oflag = 0
        lflag = 0
        cflag = (cflag | termios.CLOCAL | termios.CREAD) & ~termios.CRTSCTS
        try:
            ispeed = ospeed = termios.B115200
        except AttributeError:
            pass
        cc[termios.VMIN] = 0
        cc[termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, [iflag, oflag, cflag, lflag,
                                                ispeed, ospeed, cc])
        termios.tcflush(fd, termios.TCIOFLUSH)
    except termios.error:
        # Some CDC stacks don't support all tcsetattr fields; the fd still works.
        pass
    return fd


def _read_for(fd, seconds):
    """Drain everything the board sends for `seconds`; return decoded text."""
    end = time.time() + seconds
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
        else:
            time.sleep(0.02)
    return buf.decode("utf-8", "replace")


def send_cmd(port, cmd, wait=0.4, quiet=False):
    """Send one command line, return the board's response text."""
    if ch340_usb.is_port(port):
        dev = ch340_usb.CH340(port)
        dev.open()
        try:
            dev.write(b"\r\n")
            time.sleep(0.05)
            dev.flush_input()
            dev.write((cmd + "\r\n").encode())
            resp = dev.read_for(wait)
        finally:
            dev.close()
        return _report(resp, cmd, quiet)
    fd = open_port(port)
    try:
        # nudge the REPL to a fresh prompt, then flush its echo
        os.write(fd, b"\r\n")
        time.sleep(0.05)
        _read_for(fd, 0.05)
        os.write(fd, (cmd + "\r\n").encode())
        resp = _read_for(fd, wait)
    finally:
        os.close(fd)
    return _report(resp, cmd, quiet)


def _report(resp, cmd, quiet):
    if not quiet:
        for line in resp.splitlines():
            line = line.strip()
            if line and line != cmd and not line.startswith("beesdr-tx>"):
                print("    " + line)
    return resp


def cmd_list(args):
    ports = find_ports(args.port)
    if not ports:
        print("No ESP32 console ports found "
              "(looked for /dev/cu.usbmodem*, usbserial*, wchusbserial*).")
        print("Is a board plugged in and its USB driver present?")
        return 1
    print("Detected %d candidate port(s):" % len(ports))
    for p in ports:
        print("  " + p)
        # Try to read identity; harmless if it isn't our firmware.
        resp = send_cmd(p, "status", wait=0.5, quiet=True)
        line = next((l.strip() for l in resp.splitlines()
                     if l.strip().startswith("STATUS")), None)
        print("    " + (line if line else "(no BEESDR-TX status response)"))
    return 0


def cmd_send(args):
    ports = find_ports(args.port)
    if not ports:
        print("No ports found.")
        return 1
    for p in ports:
        print(p + ":")
        send_cmd(p, args.command, wait=getattr(args, "wait", 0.4))
    return 0


def cmd_status(args):
    args.command = "status"
    return cmd_send(args)


def cmd_mode(args):
    """Switch radio personality. The firmware reboots into the new mode, so
    wait for its console to come back before reading the status."""
    ports = find_ports(args.port)
    if not ports:
        print("No ports found.")
        return 1
    for p in ports:
        print(p + ":")
        resp = send_cmd(p, "mode %s" % args.mode, wait=0.4)
        if "rebooting" in resp:
            time.sleep(1.5)
        send_cmd(p, "status", wait=0.5)
    return 0


def cmd_assign(args):
    ports = find_ports(args.port)
    if not ports:
        print("No ESP32 ports found — nothing to assign.")
        return 1
    channels = [int(c) for c in args.channels.split(",") if c.strip()]
    if not channels:
        print("--channels is required, e.g. --channels 1,6,11")
        return 1
    for c in channels:
        if c < 1 or c > 14:
            print("Channel %d out of range 1-14" % c)
            return 1

    print("Assigning %d board(s) across channels %s (round-robin):"
          % (len(ports), channels))
    for i, p in enumerate(ports):
        ch = channels[i % len(channels)]
        print("\n%s  ->  channel %d" % (p, ch))
        send_cmd(p, "tx off", wait=0.3, quiet=True)
        send_cmd(p, "ch %d" % ch, wait=0.3)
        if args.duty is not None:
            send_cmd(p, "duty %d" % args.duty, wait=0.3)
        if args.pwr is not None:
            send_cmd(p, "pwr %d" % args.pwr, wait=0.3)
        if args.rate is not None:
            send_cmd(p, "rate %s" % args.rate, wait=0.3)
        if not args.off:
            send_cmd(p, "tx on", wait=0.3)
        send_cmd(p, "status", wait=0.5)
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", action="append",
                    help="explicit port (repeatable); default = autodetect all")
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("list", help="enumerate boards and print each one's status")

    sp = sub.add_parser("send", help="send one command to all (or --port) boards")
    sp.add_argument("command", help='e.g. "ch 11", "duty 50", "tx on"')
    sp.add_argument("--wait", type=float, default=0.4)

    sub.add_parser("status", help="print status from all (or --port) boards")

    sm = sub.add_parser("mode", help="switch boards between WiFi and BLE (reboots them)")
    sm.add_argument("mode", choices=["wifi", "ble"])

    sa = sub.add_parser("assign",
                        help="round-robin channels across boards and start TX")
    sa.add_argument("--channels", required=True,
                    help="comma list, e.g. 1,6,11 (round-robin if fewer than boards)")
    sa.add_argument("--duty", type=int, help="0-100 on-air percent")
    sa.add_argument("--pwr", type=int, help="2-20 dBm")
    sa.add_argument("--rate", help="PHY rate, selects the standard: b=1m..11m, "
                    "g=6m(default)..54m, n=mcs0..mcs7, ax=he-mcs0..he-mcs9")
    sa.add_argument("--off", action="store_true",
                    help="configure but leave TX paused (tx off)")

    args = ap.parse_args()
    if args.cmd == "list":
        return cmd_list(args)
    if args.cmd == "send":
        return cmd_send(args)
    if args.cmd == "status":
        return cmd_status(args)
    if args.cmd == "mode":
        return cmd_mode(args)
    if args.cmd == "assign":
        return cmd_assign(args)
    return 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except OSError as e:
        sys.exit("%s" % e)
