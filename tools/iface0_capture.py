#!/usr/bin/env python3
"""Read the ZD Controller (2345:e023) vendor interface 0 via libusb.

Interface 0 (Vendor Specific class, EP 0x81 IN) is NOT bound by the kernel
(no hidraw node), so it has to be read directly with libusb. This channel is
expected to carry the full Xinput-style data: analog triggers, possibly IMU
(gyro/accel) once enabled.

Permission: needs write access to /dev/bus/usb — either run as root once or
install tools/91-zd-controller.rules (then replug the receiver).

Usage:
    python3 tools/iface0_capture.py [duration_s] [outfile]
"""
import sys
import time

import usb.core
import usb.util

EP_IN = 0x81
EP_OUT = 0x02


def main():
    try:
        duration = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
    except ValueError:
        sys.exit("invalid duration: %r" % sys.argv[1])
    out = None
    if len(sys.argv) > 2:
        try:
            out = open(sys.argv[2], "a")
        except OSError as e:
            sys.exit("cannot open %s: %s" % (sys.argv[2], e))

    # pyusb's stub marks find() as a Generator; it actually returns Device|None.
    dev = usb.core.find(idVendor=0x2345, idProduct=0xe023)  # type: ignore[assignment]
    if dev is None:
        sys.exit("ZD Controller not found — is the receiver plugged in?")

    # Device already runs config 1 — do NOT call set_configuration: it can
    # return EBUSY because usbhid holds interface 1 (kernel hot-unbind).
    try:
        usb.util.claim_interface(dev, 0)  # type: ignore[attr-defined]
    except usb.core.USBError as e:
        sys.exit("claim_interface(0) failed: %s" % e)

    def emit(s):
        print(s, flush=True)
        if out:
            out.write(s + "\n")
            out.flush()

    emit("# iface0 capture  %s  (EP 0x81)" % time.strftime("%Y-%m-%d %H:%M:%S"))
    end = time.time() + duration
    last = None
    n = 0
    while time.time() < end:
        try:
            data = dev.read(EP_IN, 64, timeout=1000)  # type: ignore[attr-defined]
        except usb.core.USBError as e:
            if e.errno == 110:  # timeout — fine, keep polling
                continue
            emit("read error: %s" % e)
            break
        n += 1
        b = bytes(data)
        if last is None:
            emit("%s  #%d  len=%d  %s  (first frame)" % (time.strftime("%H:%M:%S"), n, len(b), b.hex(" ")))
            last = b
            continue
        if b == last:
            continue
        diffs = [i for i in range(min(len(b), len(last))) if b[i] != last[i]]
        tag = ", ".join("%d=%02x" % (i, b[i]) for i in diffs)
        emit("%s  #%d  CHANGED: %s" % (time.strftime("%H:%M:%S"), n, tag))
        last = b

    usb.util.release_interface(dev, 0)
    if out:
        out.close()
    print("captured %d frames in %.1fs" % (n, duration))


if __name__ == "__main__":
    main()
