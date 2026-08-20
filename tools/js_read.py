#!/usr/bin/env python3
"""Realtime view of the xpad-mapped ZD Controller (2345:e023) as /dev/input/js1.

Prints live axis/button changes only. xpad maps this device as "Generic X-Box pad":
  axis 0=LX 1=LY 2=LT 3=RX 4=RY 5=RT 6=hatX 7=hatY
  btn 0=A 1=B 2=X 3=Y 4=LB 5=RB 6=BACK 7=START 8=GUIDE 9=L3 10=R3

Usage: python3 tools/js_read.py [duration_s]
"""
import os
import select
import struct
import sys
import time

AXES = {0: "LX", 1: "LY", 2: "LT", 3: "RX", 4: "RY", 5: "RT", 6: "hatX", 7: "hatY"}
BTNS = {0: "A", 1: "B", 2: "X", 3: "Y", 4: "LB", 5: "RB", 6: "BACK", 7: "START", 8: "GUIDE", 9: "L3", 10: "R3"}

try:
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
except ValueError:
    sys.exit("invalid duration: %r" % sys.argv[1])

try:
    fd = os.open("/dev/input/js1", os.O_RDONLY | os.O_NONBLOCK)
except OSError as e:
    sys.exit("cannot open /dev/input/js1: %s (device may have moved; check /dev/input/by-id)" % e)

end = time.time() + duration
print("reading /dev/input/js1 for %.1fs — move sticks / triggers / press buttons" % duration)
while time.time() < end:
    r, _, _ = select.select([fd], [], [], 0.2)
    if not r:
        continue
    d = os.read(fd, 8)
    if len(d) != 8:
        continue
    _t, val, typ, num = struct.unpack("<IhBB", d)
    if typ & 0x80:
        continue  # INIT snapshot, skip
    if typ & 0x02:
        name = AXES.get(num, "axis%d" % num)
        print("%s  %-5s = %+6d" % (time.strftime("%H:%M:%S"), name, val), flush=True)
    elif typ & 0x01:
        name = BTNS.get(num, "btn%d" % num)
        if val:
            print("%s  %-5s DOWN" % (time.strftime("%H:%M:%S"), name), flush=True)

os.close(fd)
print("done")
