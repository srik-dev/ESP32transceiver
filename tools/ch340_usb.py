#!/usr/bin/env python3
"""
ch340_usb.py — userspace driver for WCH CH340 USB-UART bridges that macOS has no
kernel driver for, so they never get a /dev/cu.* entry.

Apple's built-in driver matches only 1a86:7523 and 1a86:55d4. A board whose
bridge is a CH340K (1a86:7522) enumerates on USB and then just sits there. This
talks to it directly through libusb instead, so no driver install (admin +
system-extension approval) is needed.

Pure stdlib: ctypes against Homebrew's libusb (`brew install libusb`). No pyusb,
no pip — the dashboard and esp32tx_ctl.py import it on the system python3.

Ports are named by USB topology, which is stable across replugs into the same
socket:   ch340:<bus>-<port>[.<port>...]      e.g. ch340:0-1.2

As a library:
    import ch340_usb
    for p in ch340_usb.list_ports(): ...
    dev = ch340_usb.CH340(p); dev.open()
    dev.write(b"status\\r\\n"); print(dev.read(4096, timeout_ms=300))

As a tool:
    tools/ch340_usb.py list
    tools/ch340_usb.py reset   [PORT]            # reboot the board, print boot log
    tools/ch340_usb.py esptool [PORT] <args...>  # run esptool over the bridge
                                                 # (needs esptool: IDF python env)

Only one process can hold a bridge at a time — same rule as a serial port.
"""
import ctypes
import ctypes.util
import os
import sys
import threading
import time

PREFIX = "ch340:"
VID = 0x1A86
# Only PIDs macOS leaves driverless. 0x7523 / 0x55d4 already get a /dev/cu.* port
# from Apple's driver, and claiming them here would fight it.
PIDS = (0x7522,)

_ERR_TIMEOUT = -7
_ERRS = {-1: "I/O error", -3: "access denied (held by another process — "
         "is the dashboard service running?)", -4: "device unplugged",
         -5: "not found", -6: "busy (held by another process)", -9: "pipe error"}
_OUT, _IN = 0x40, 0xC0
_CLK = 48000000


class _DevDesc(ctypes.Structure):
    _fields_ = [("bLength", ctypes.c_uint8), ("bDescriptorType", ctypes.c_uint8),
                ("bcdUSB", ctypes.c_uint16), ("bDeviceClass", ctypes.c_uint8),
                ("bDeviceSubClass", ctypes.c_uint8), ("bDeviceProtocol", ctypes.c_uint8),
                ("bMaxPacketSize0", ctypes.c_uint8), ("idVendor", ctypes.c_uint16),
                ("idProduct", ctypes.c_uint16), ("bcdDevice", ctypes.c_uint16),
                ("iManufacturer", ctypes.c_uint8), ("iProduct", ctypes.c_uint8),
                ("iSerialNumber", ctypes.c_uint8), ("bNumConfigurations", ctypes.c_uint8)]


_lib = None
_ctx = ctypes.c_void_p()
_init_lock = threading.Lock()


def _libusb():
    """Load libusb once; None if it is not installed."""
    global _lib
    with _init_lock:
        if _lib is not None:
            return _lib or None
        cands = ["/opt/homebrew/lib/libusb-1.0.dylib", "/usr/local/lib/libusb-1.0.dylib",
                 ctypes.util.find_library("usb-1.0")]
        lib = None
        for c in cands:
            if not c:
                continue
            try:
                lib = ctypes.CDLL(c)
                break
            except OSError:
                continue
        if lib is None:
            _lib = False
            return None
        vp, u8, u16, ci, cu = (ctypes.c_void_p, ctypes.c_uint8, ctypes.c_uint16,
                               ctypes.c_int, ctypes.c_uint)
        P = ctypes.POINTER
        lib.libusb_init.argtypes = [P(vp)]
        lib.libusb_get_device_list.argtypes = [vp, P(P(vp))]
        lib.libusb_get_device_list.restype = ctypes.c_ssize_t
        lib.libusb_free_device_list.argtypes = [P(vp), ci]
        lib.libusb_free_device_list.restype = None
        lib.libusb_get_device_descriptor.argtypes = [vp, P(_DevDesc)]
        lib.libusb_get_bus_number.argtypes = [vp]
        lib.libusb_get_bus_number.restype = u8
        lib.libusb_get_port_numbers.argtypes = [vp, P(u8), ci]
        lib.libusb_open.argtypes = [vp, P(vp)]
        lib.libusb_close.argtypes = [vp]
        lib.libusb_close.restype = None
        lib.libusb_set_configuration.argtypes = [vp, ci]
        lib.libusb_claim_interface.argtypes = [vp, ci]
        lib.libusb_release_interface.argtypes = [vp, ci]
        lib.libusb_control_transfer.argtypes = [vp, u8, u8, u16, u16, ctypes.c_char_p, u16, cu]
        lib.libusb_bulk_transfer.argtypes = [vp, u8, ctypes.c_char_p, ci, P(ci), cu]
        if lib.libusb_init(ctypes.byref(_ctx)) != 0:
            _lib = False
            return None
        _lib = lib
        return lib


def _name(lib, dev):
    ports = (ctypes.c_uint8 * 8)()
    n = lib.libusb_get_port_numbers(dev, ports, 8)
    path = ".".join(str(ports[i]) for i in range(max(n, 0)))
    return "%s%d-%s" % (PREFIX, lib.libusb_get_bus_number(dev), path)


def _walk(fn):
    """Call fn(lib, dev, name) for each driverless CH340; stop at a truthy result."""
    lib = _libusb()
    if lib is None:
        return None
    lst = ctypes.POINTER(ctypes.c_void_p)()
    n = lib.libusb_get_device_list(_ctx, ctypes.byref(lst))
    if n < 0:
        return None
    try:
        for i in range(n):
            d = _DevDesc()
            if lib.libusb_get_device_descriptor(lst[i], ctypes.byref(d)) != 0:
                continue
            if d.idVendor == VID and d.idProduct in PIDS:
                r = fn(lib, lst[i], _name(lib, lst[i]))
                if r:
                    return r
    finally:
        lib.libusb_free_device_list(lst, 1)
    return None


def list_ports():
    """Names of every attached driverless CH340, sorted. [] without libusb."""
    out = []
    _walk(lambda lib, dev, name: out.append(name))
    return sorted(out)


def is_port(name):
    return isinstance(name, str) and name.startswith(PREFIX)


def _divisor(speed):
    """Baud -> CH340 prescaler/divisor register value (as Linux ch341.c)."""
    speed = max(46, min(int(speed), 2000000))
    fact = 1
    for ps in (3, 2, 1, 0):
        if speed > _CLK // ((1 << (12 - 3 * ps - 1)) * 512):
            break
    cd = 1 << (12 - 3 * ps - fact)
    div = _CLK // (cd * speed)
    if div < 9 or div > 255:
        div //= 2
        cd *= 2
        fact = 0
    if 16 * _CLK // (cd * div) - 16 * speed >= 16 * speed - 16 * _CLK // (cd * (div + 1)):
        div += 1
    if fact == 1 and div % 2 == 0:
        div //= 2
        fact = 0
    return ((0x100 - div) << 8) | (fact << 2) | ps


class CH340:
    """One bridge. Errors are raised as OSError, like a tty would."""

    def __init__(self, port=None, baud=115200):
        self.port = port          # None -> first one found
        self.baud = baud
        self._h = None
        self._lib = None
        self._dtr = False
        self._rts = False

    def _check(self, rc, what):
        if rc < 0:
            raise OSError("%s: %s: %s" % (self.port or "ch340", what,
                                          _ERRS.get(rc, "libusb error %d" % rc)))
        return rc

    def open(self):
        want = self.port

        def grab(lib, dev, name):
            if want and name != want:
                return None
            h = ctypes.c_void_p()
            return (lib, h, name, lib.libusb_open(dev, ctypes.byref(h)))

        got = _walk(grab)
        if not got:
            raise OSError("%s: no such CH340 bridge%s" % (
                want or "ch340", "" if _libusb() else " (libusb not installed)"))
        lib, h, self.port, rc = got
        self._lib = lib
        self._check(rc, "open")
        self._h = h
        try:
            lib.libusb_set_configuration(h, 1)          # may already be set
            self._check(lib.libusb_claim_interface(h, 0), "claim")
            self._ctrl(0xA1, 0, 0)                      # serial init
            self.set_baud(self.baud)
            self.set_lines(False, False)                # both idle: board keeps running
        except OSError:
            self.close()
            raise

    def close(self):
        h, self._h = self._h, None
        if h is not None:
            self._lib.libusb_release_interface(h, 0)
            self._lib.libusb_close(h)

    def _ctrl(self, req, value, index):
        if self._h is None:
            raise OSError("%s: closed" % self.port)
        self._check(self._lib.libusb_control_transfer(
            self._h, _OUT, req, value & 0xFFFF, index & 0xFFFF, None, 0, 1000), "control")

    def set_baud(self, baud):
        self.baud = baud
        self._ctrl(0x9A, 0x1312, _divisor(baud) | 0x80)
        self._ctrl(0x9A, 0x2518, 0xC3)                  # 8N1, rx + tx enabled

    def set_lines(self, dtr=None, rts=None):
        """Assert/deassert DTR and RTS (True = asserted = pin driven low)."""
        if dtr is not None:
            self._dtr = bool(dtr)
        if rts is not None:
            self._rts = bool(rts)
        c = (0x20 if self._dtr else 0) | (0x40 if self._rts else 0)
        self._ctrl(0xA4, ~c, 0)

    def reset(self, bootloader=False):
        """Pulse EN through the usual DTR/RTS auto-reset circuit; with
        bootloader=True hold IO0 low across it so the ROM enters download mode."""
        self.set_lines(False, True)                     # EN low
        time.sleep(0.1)
        if bootloader:
            self.set_lines(True, False)                 # IO0 low, EN high
            time.sleep(0.06)
            self.set_lines(False, False)
        else:
            self.set_lines(False, False)

    def read(self, size=4096, timeout_ms=20):
        """Up to `size` bytes; b"" if nothing arrived within the timeout."""
        if self._h is None:
            raise OSError("%s: closed" % self.port)
        buf = ctypes.create_string_buffer(max(size, 64))
        n = ctypes.c_int(0)
        rc = self._lib.libusb_bulk_transfer(self._h, 0x82, buf, len(buf),
                                            ctypes.byref(n), max(int(timeout_ms), 1))
        if rc != _ERR_TIMEOUT:
            self._check(rc, "read")
        return buf.raw[:n.value]

    def write(self, data):
        if self._h is None:
            raise OSError("%s: closed" % self.port)
        data = bytes(data)
        off = 0
        while off < len(data):
            chunk = data[off:off + 4096]
            n = ctypes.c_int(0)
            self._check(self._lib.libusb_bulk_transfer(
                self._h, 0x02, chunk, len(chunk), ctypes.byref(n), 5000), "write")
            off += n.value
        return len(data)

    def flush_input(self):
        for _ in range(64):
            if not self.read(4096, 2):
                break

    def read_for(self, seconds):
        """Everything the board sends over `seconds`, decoded."""
        end = time.time() + seconds
        buf = bytearray()
        while time.time() < end:
            buf.extend(self.read(4096, 20))
        return buf.decode("utf-8", "replace")


# ---------------------------------------------------------------- esptool ---

def _pyserial_class():
    """A pyserial-compatible port over CH340, for esptool. Needs pyserial, so it
    is built on demand — everything above stays stdlib-only."""
    from serial.serialutil import SerialBase, Timeout, to_bytes

    class Serial(SerialBase):
        def open(self):
            self._ch = CH340(self._port, self._baudrate)
            self._ch.open()
            self._buf = bytearray()
            self._cv = threading.Condition()
            self._run = True
            self.is_open = True
            # pyserial defaults both lines to asserted. Left that way, esptool's
            # RTS-pulse hard reset never pulls EN low and ends with IO0 held low,
            # stranding the chip in the bootloader. Start from idle instead.
            self._dtr_state = self._rts_state = False
            self._ch.set_lines(False, False)
            self._thr = threading.Thread(target=self._reader, daemon=True)
            self._thr.start()

        def _reader(self):
            while self._run:
                try:
                    d = self._ch.read(4096, 50)
                except OSError:
                    break
                if d:
                    with self._cv:
                        self._buf.extend(d)
                        self._cv.notify_all()

        def _reconfigure_port(self):
            if self.is_open:
                self._ch.set_baud(self._baudrate)

        def _update_dtr_state(self):
            self._ch.set_lines(dtr=self._dtr_state)

        def _update_rts_state(self):
            self._ch.set_lines(rts=self._rts_state)

        def _update_break_state(self):
            pass

        def close(self):
            if self.is_open:
                self._run = False
                self._thr.join(0.5)
                self._ch.close()
                self.is_open = False

        @property
        def in_waiting(self):
            with self._cv:
                return len(self._buf)

        def read(self, size=1):
            t = Timeout(self._timeout)
            with self._cv:
                while len(self._buf) < size and not t.expired():
                    left = t.time_left()
                    self._cv.wait(0.05 if left is None else min(0.05, max(left, 0.001)))
                out = bytes(self._buf[:size])
                del self._buf[:size]
            return out

        def write(self, data):
            return self._ch.write(to_bytes(data))

        def flush(self):
            pass

        def reset_input_buffer(self):
            with self._cv:
                self._buf.clear()

        def reset_output_buffer(self):
            pass

        cts = dsr = ri = cd = property(lambda self: False)

    return Serial


def run_esptool(port, args):
    """Run esptool against a bridge. The bootloader is entered here (esptool's
    own reset strategies poke a tty fd, which this is not), so esptool is told
    --before no_reset; its --after hard_reset works through set_lines."""
    import serial
    import esptool
    cls = _pyserial_class()
    orig = serial.serial_for_url

    def for_url(url, *a, **kw):
        if not is_port(url):
            return orig(url, *a, **kw)
        kw.pop("do_not_open", None)
        s = cls(None, *a, **kw)
        s.port = url
        return s

    serial.serial_for_url = for_url
    port = port or (list_ports() or [None])[0]
    if not port:
        sys.exit("no driverless CH340 bridge found")
    ch = CH340(port)
    ch.open()
    ch.reset(bootloader=True)
    time.sleep(0.2)
    ch.close()
    sys.argv = ["esptool", "-p", port, "--before", "no_reset"] + list(args)
    esptool._main()


def main(argv):
    if not argv or argv[0] in ("-h", "--help"):
        print(__doc__)
        return 0
    cmd, rest = argv[0], argv[1:]
    port = rest.pop(0) if rest and is_port(rest[0]) else None
    if cmd == "list":
        for p in list_ports():
            print(p)
        return 0
    if cmd == "reset":
        ch = CH340(port)
        ch.open()
        ch.reset()
        sys.stdout.write(ch.read_for(float(os.environ.get("T", "3"))))
        ch.close()
        return 0
    if cmd == "esptool":
        run_esptool(port, rest)
        return 0
    print("unknown command %r (list | reset | esptool)" % cmd, file=sys.stderr)
    return 2


if __name__ == "__main__":
    try:
        sys.exit(main(sys.argv[1:]))
    except OSError as e:
        sys.exit(str(e))
