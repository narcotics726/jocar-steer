#!/usr/bin/env python3
"""Decode the jocar-steer persistent flash event log.

The firmware stores append-only 64-byte records in the unused flash sector
right after the app partition (see src/flash_log.rs). Reading it back does not
need the console to have been attached while the events happened:

    ~/.cargo/bin/espflash read-flash 0x00FB0000 0x1000 /tmp/flashlog.bin
    python3 tools/flashlog_decode.py /tmp/flashlog.bin

(esptool works too: `esptool.py read_flash 0x00FB0000 0x1000 /tmp/flashlog.bin`.)
"""

import struct
import sys

MAGIC = 0x474C434A  # "JCLG"
REC_SIZE = 64
HEADER = 16

KINDS = {
    1: "BOOT",
    2: "CONNECTED",
    3: "ENUM_FAIL",
    4: "LOST",
    5: "RESET",
    6: "ENUM_OK",
    7: "IFACE",
    8: "HUB",
    9: "SESSION",
    10: "READ_ERR",
    11: "STALE",
    12: "FIRST_REPORT",
    13: "PARSE_FAIL",
}


def decode(path):
    data = open(path, "rb").read()
    shown = 0
    for i in range(len(data) // REC_SIZE):
        rec = data[i * REC_SIZE : (i + 1) * REC_SIZE]
        magic, seq, slot, kind = struct.unpack_from("<IIII", rec, 0)
        if magic != MAGIC:
            continue
        text = rec[HEADER:].split(b"\x00", 1)[0].decode("utf-8", "replace")
        name = KINDS.get(kind, f"KIND{kind}")
        print(f"seq={seq:<6} slot={slot:<4} {name:<10} {text}")
        shown += 1
    print(f"--- {shown} record(s), {len(data)} bytes read ---")


if __name__ == "__main__":
    decode(sys.argv[1] if len(sys.argv) > 1 else "/tmp/flashlog.bin")
