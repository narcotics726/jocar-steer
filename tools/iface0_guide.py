#!/usr/bin/env python3
"""ZD Controller (2345:e023) interface-0 interactive byte-mapping guide.

Reads the vendor interface 0 (EP 0x81, 64-byte frames) via libusb and walks
through each input one at a time, waiting for your presses — no flood of
frames, full control of the pace.

Detects per action group:
  LT/RT     -> byte 4/5 (analog triggers, Xbox360 style)
  sticks    -> candidate bytes {6..9, 16..19} (16-bit LE pairs vs 8-bit)
  buttons   -> candidate bytes {2,3,13,14,15}
  motion    -> every byte, esp. the drifting ones (gyro?)

Usage:
  python3 tools/iface0_guide.py
  (press Enter to skip an item, Ctrl-C to end; report written to /tmp/iface0_guide_*.txt)
"""
import os
import select
import sys
import time

import usb.core
import usb.util

EP_IN = 0x81
TRIG = {4: "LT", 5: "RT"}
# Xbox360 region (6..13) + embedded 8-bit region (20..23); 16..19 is header+buttons.
STICK = [6, 7, 8, 9, 10, 11, 12, 13, 20, 21, 22, 23]
BTNS = [2, 3, 13, 14, 15]


def find_dev():
    dev = usb.core.find(idVendor=0x2345, idProduct=0xe023)  # type: ignore[assignment]
    if dev is None:
        sys.exit("ZD Controller not found — receiver plugged in?")
    try:
        usb.util.claim_interface(dev, 0)  # type: ignore[attr-defined]
    except usb.core.USBError as e:
        sys.exit("claim_interface(0) failed (xpad still bound? unbind it first): %s" % e)
    return dev


class Guide:
    def __init__(self, dev, out):
        self.dev = dev
        self.out = out
        self.base = None
        os.set_blocking(sys.stdin.fileno(), False)

    def emit(self, s):
        print(s, flush=True)
        self.out.write(s + "\n")
        self.out.flush()

    def read_frame(self, timeout=60):
        try:
            data = self.dev.read(EP_IN, 64, timeout=timeout)  # type: ignore[attr-defined]
            return bytes(data)
        except usb.core.USBError as e:
            if e.errno == 110:
                return None
            raise

    def skip_or_key(self):
        r, _, _ = select.select([sys.stdin.fileno()], [], [], 0)
        if r:
            os.read(sys.stdin.fileno(), 4096)
            return True
        return False

    # ── baseline ──
    def baseline(self, n=12):
        self.emit("── 基线（松开手柄，静止 2 秒）")
        frames = []
        end = time.time() + 3.0
        while len(frames) < n and time.time() < end:
            f = self.read_frame()
            if f is not None:
                frames.append(f)
        if not frames:
            sys.exit("no frames — is the receiver in e023 (blue) mode?")
        self.base = frames[-1]
        self.emit("  基线帧: %s" % self.base.hex(" "))
        for i in range(0, 64, 4):
            chunk = self.base[i:i + 4]
            if any(chunk):
                self.emit("  [%2d..%2d] %s" % (i, i + 3, " ".join("%02x" % b for b in chunk)))

    def delta(self, f, idx):
        assert self.base is not None
        return abs(f[idx] - self.base[idx])

    # ── analog trigger ──
    def trig(self, name, idx):
        assert self.base is not None
        self.emit("")
        self.emit("▶ %s（byte %d）" % (name, idx))
        self.emit("  按住 [%s] 至半程并保持 1 秒，然后松开（回车跳过）" % name)
        pressed = None
        while pressed is None:
            if self.skip_or_key():
                self.emit("  已跳过")
                return
            f = self.read_frame()
            if f is not None and self.delta(f, idx) > 30:
                pressed = f
                self.emit("  按下: %d = %d (0x%02x)" % (idx, f[idx], f[idx]))
                self.emit("  请松开……")
        while True:
            if self.skip_or_key():
                return
            f = self.read_frame()
            if f is not None and self.delta(f, idx) < 15:
                self.emit("  已松开 ✓ 回到 %d" % self.base[idx])
                return

    # ── stick (candidates 6..9 and 16..19) ──
    def stick(self, name):
        assert self.base is not None
        self.emit("")
        self.emit("▶ %s" % name)
        self.emit("  将 [%s] 推到底并保持 1.5 秒，然后松开（回车跳过）" % name)
        while True:
            if self.skip_or_key():
                self.emit("  已跳过")
                return
            f = self.read_frame()
            if f is not None and any(self.delta(f, i) > 60 for i in STICK):
                self.emit("  检测到偏移，稳定中……")
                break
        t0 = time.time()
        while time.time() - t0 < 0.3:
            self.read_frame()
        mins = {i: 255 for i in STICK}
        maxs = {i: 0 for i in STICK}
        t0 = time.time()
        stable = []
        while time.time() - t0 < 1.5:
            f = self.read_frame()
            if f is not None:
                for i in STICK:
                    mins[i] = min(mins[i], f[i])
                    maxs[i] = max(maxs[i], f[i])
                stable.append(f)
        self.emit("  采样 min..max: %s" % "  ".join("%d=%d..%d" % (i, mins[i], maxs[i]) for i in STICK if maxs[i] - mins[i] >= 1))
        if stable:
            s = stable[-1]  # last stable frame
            moved = [i for i in STICK if abs(s[i] - self.base[i]) > 15]
            self.emit("  末帧: %s" % "  ".join("%d=%d" % (i, s[i]) for i in moved))
        self.emit("  请松开（回车跳过等待）")
        while True:
            if self.skip_or_key():
                return
            f = self.read_frame()
            if f is not None and not any(self.delta(f, i) > 40 for i in STICK):
                self.emit("  已回中 ✓")
                return

    # ── button ──
    def btn(self, name):
        assert self.base is not None
        self.emit("")
        self.emit("▶ %s" % name)
        self.emit("  按一下 [%s] 并松开（回车跳过）" % name)
        pressed = None
        while pressed is None:
            if self.skip_or_key():
                self.emit("  已跳过")
                return
            f = self.read_frame()
            if f is not None and any(f[i] != self.base[i] for i in BTNS):
                pressed = f
                changed = [i for i in BTNS if f[i] != self.base[i]]
                self.emit("  按下: %s" % " ".join("%d=%02x" % (i, f[i]) for i in changed))
                self.emit("  请松开……")
        while True:
            if self.skip_or_key():
                return
            f = self.read_frame()
            if f is not None and all(f[i] == self.base[i] for i in BTNS):
                self.emit("  已松开 ✓")
                return

    # ── hold button (R3 etc: avoid stick noise while pressed) ──
    def hold_btn(self, name):
        assert self.base is not None
        self.emit("")
        self.emit("▶ %s" % name)
        self.emit("  按住 [%s] 保持 1 秒（尽量别带动摇杆），然后松开（回车跳过）" % name)
        pressed = None
        while pressed is None:
            if self.skip_or_key():
                self.emit("  已跳过")
                return
            f = self.read_frame()
            if f is not None and any(f[i] != self.base[i] for i in BTNS):
                pressed = f
                changed = [i for i in BTNS if f[i] != self.base[i]]
                self.emit("  按下: %s" % " ".join("%d=%02x" % (i, f[i]) for i in changed))
                # sample while held
                mins = {i: 255 for i in BTNS}
                maxs = {i: 0 for i in BTNS}
                t0 = time.time()
                while time.time() - t0 < 1.0:
                    f = self.read_frame()
                    if f is not None:
                        for i in BTNS:
                            mins[i] = min(mins[i], f[i])
                            maxs[i] = max(maxs[i], f[i])
                stable = [i for i in BTNS if maxs[i] - mins[i] < 2 and maxs[i] != self.base[i]]
                if stable:
                    self.emit("  稳定按钮位: %s" % " ".join("%d=0x%02x" % (i, maxs[i]) for i in stable))
                else:
                    self.emit("  注意：无稳定按钮位，变化可能来自摇杆噪声")
                self.emit("  请松开……")
        while True:
            if self.skip_or_key():
                return
            f = self.read_frame()
            if f is not None and all(f[i] == self.base[i] for i in BTNS):
                self.emit("  已松开 ✓")
                return

    # ── motion (gyro?) ──
    def motion(self):
        assert self.base is not None
        self.emit("")
        self.emit("▶ 转动手柄（陀螺仪探测）")
        self.emit("  快速转动手柄 2 秒（绕 Z 轴左右摇），然后停住（回车跳过）")
        while True:
            if self.skip_or_key():
                self.emit("  已跳过")
                return
            f = self.read_frame()
            if f is not None and any(self.delta(f, i) > 25 for i in range(64) if i not in STICK and i not in TRIG and i not in BTNS):
                self.emit("  检测到运动，采样 2 秒……")
                break
        mins = {i: 255 for i in range(64) if self.base[i] != 0}
        maxs = {i: 0 for i in mins}
        t0 = time.time()
        while time.time() - t0 < 2.0:
            f = self.read_frame()
            if f is not None:
                for i in mins:
                    mins[i] = min(mins[i], f[i])
                    maxs[i] = max(maxs[i], f[i])
        active = [i for i in mins if maxs[i] - mins[i] >= 3]
        self.emit("  运动期间变化字节: %s" % "  ".join("%d=%d..%d" % (i, mins[i], maxs[i]) for i in active))
        self.emit("  请停住手柄（回车跳过等待）")
        while True:
            if self.skip_or_key():
                return
            f = self.read_frame()
            if f is not None and not any(self.delta(f, i) > 25 for i in active):
                self.emit("  已稳定 ✓")
                return


def main():
    dev = find_dev()
    stamp = time.strftime("%Y%m%d_%H%M%S")
    outfile = "/tmp/iface0_guide_%s.txt" % stamp
    try:
        out = open(outfile, "w")
    except OSError as e:
        sys.exit("cannot write %s: %s" % (outfile, e))
    g = Guide(dev, out)
    print("接口 0 引导捕获开始，输出: %s\n回车跳过当前项，Ctrl-C 结束。\n" % outfile)
    g.emit("# iface0 guide %s" % time.strftime("%Y-%m-%d %H:%M:%S"))
    try:
        g.baseline()
        g.trig("LT 半按", 4)
        g.trig("RT 半按", 5)
        for d in ["左摇杆 左", "左摇杆 右", "左摇杆 上", "左摇杆 下"]:
            g.stick(d)
        for d in ["右摇杆 左", "右摇杆 右", "右摇杆 上", "右摇杆 下"]:
            g.stick(d)
        g.btn("A")
        g.btn("B")
        for b in ["X", "Y", "LB", "RB", "L3", "BACK", "START"]:
            g.btn(b)
        g.hold_btn("R3（按住 1 秒，不要带动摇杆）")
        g.btn("HOME")
        for b in ["十字UP", "十字DOWN", "十字LEFT", "十字RIGHT"]:
            g.btn(b)
        g.motion()
    except KeyboardInterrupt:
        g.emit("== 用户中断 ==")
    out.close()
    print("\n完成！报告: %s\n发出来即可。" % outfile)


if __name__ == "__main__":
    main()
