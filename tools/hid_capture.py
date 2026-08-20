#!/usr/bin/env python3
"""Realtime ZD Controller (2345:e023) 32-byte vendor report capture.

Reads /dev/hidraw<X> (no root needed if udev ACL present) and prints every
report with a diff marker showing which bytes changed vs the previous frame.

Usage:
    hid_capture.py [hidraw_path] [duration_s] [outfile]
    default path /dev/hidraw9, duration 8 s, output stdout (append to file if given)
"""
import os
import select
import sys
import time

path = sys.argv[1] if len(sys.argv) > 1 else "/dev/hidraw9"
try:
    duration = float(sys.argv[2]) if len(sys.argv) > 2 else 8.0
except ValueError:
    sys.exit("invalid duration: %r" % sys.argv[2])
out = None
if len(sys.argv) > 3:
    try:
        out = open(sys.argv[3], "a")
    except OSError as e:
        sys.exit("cannot open output file %r: %s" % (sys.argv[3], e))

try:
    fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
except OSError as e:
    sys.exit("cannot open %r: %s" % (path, e))
end = time.time() + duration
last = None
n = 0

def emit(s):
    print(s, flush=True)
    if out:
        out.write(s + "\n")
        out.flush()

while time.time() < end:
    r, _, _ = select.select([fd], [], [], 0.05)
    if r:
        data = os.read(fd, 64)
        n += 1
        if len(data) == 32 and data[0] == 0x04:
            tag = ""
            if last is not None and data != last:
                diffs = [i for i in range(32) if data[i] != last[i]]
                tag = "  CHANGED: " + ", ".join("%d=%02x" % (i, data[i]) for i in diffs)
            elif last is None:
                tag = "  (first frame)"
            emit("%s  #%d  %s%s" % (time.strftime("%H:%M:%S"), n, data.hex(" "), tag))
            last = data

os.close(fd)
if out:
    out.close()
print("captured %d frames in %.1fs" % (n, duration))
