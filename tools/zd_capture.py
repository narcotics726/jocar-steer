#!/usr/bin/env python3
"""ZD 手柄 (2345:e023) 键位映射交互式捕获向导。

自动寻找 ZD 接收器的 hidraw 节点，逐个提示按键：
  · 按钮：按下 → 保持 ~0.5s → 松开，检测到按下/松开后自动进入下一步
  · 摇杆：推到底并保持 ~1.5s（脚本采样范围），然后松开回中
  · 扳机：同时检测模拟量或按钮位（两种映射都能抓到）
  · 十字键：若 [3][4] 位图无反应，会提示检查键盘 report（可能走键盘接口）

运行:  python3 tools/zd_capture.py
交互:  按屏幕提示操作；想跳过当前项按回车；随时 Ctrl-C 结束（已记录保留）。
输出:  /tmp/zd_map_<时间>.txt —— 跑完把文件内容发出来即可。
"""
import glob
import os
import select
import sys
import time

SIM = {6: "LX", 7: "LY", 8: "RX", 9: "RY"}  # 已知 4 个模拟通道


def find_hidraw():
    for p in sorted(glob.glob("/sys/bus/hid/devices/0003:2345:*/hidraw/*")):
        node = "/dev/" + os.path.basename(p)
        if os.path.exists(node):
            return node
    nodes = sorted(glob.glob("/dev/hidraw*"))
    if nodes:
        print("未自动找到 ZD 节点，可用: %s" % " ".join(nodes))
    return None


class Capture:
    def __init__(self, path, out):
        try:
            self.fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
        except OSError as e:
            sys.exit("无法打开 %s: %s" % (path, e))
        self.out = out
        self.base = None          # 基线 32B 帧
        self.non32 = {}           # len -> [count, sample_hex]
        self.total = 0
        os.set_blocking(sys.stdin.fileno(), False)

    def emit(self, s):
        print(s, flush=True)
        self.out.write(s + "\n")
        self.out.flush()

    def read_one(self, timeout):
        """读一帧；返回 32B 厂商帧或 None（非厂商帧计入 non32）。"""
        r, _, _ = select.select([self.fd], [], [], timeout)
        if not r:
            return None
        d = os.read(self.fd, 64)
        self.total += 1
        if len(d) == 32 and d[0] == 0x04:
            return d
        key = len(d)
        if key not in self.non32:
            self.non32[key] = [0, d.hex(" ")]
        self.non32[key][0] += 1
        return None

    def report_non32(self):
        if self.non32:
            parts = []
            for k, (n, sample) in sorted(self.non32.items()):
                parts.append("len=%d ×%d 样例 %s" % (k, n, sample))
            self.emit("  [提示] 期间收到非厂商 report: %s" % " ; ".join(parts))

    def skip_or_key(self):
        """用户按回车则返回 True（跳过当前项）。"""
        r, _, _ = select.select([sys.stdin.fileno()], [], [], 0)
        if r:
            os.read(sys.stdin.fileno(), 4096)
            return True
        return False

    # ── 基线 ──
    def capture_baseline(self, n=15):
        self.emit("── 步骤 0：采集基线（请完全松开手柄，保持静止，勿触碰）")
        frames = []
        while len(frames) < n:
            d = self.read_one(2.0)
            if d is not None:
                frames.append(d)
        self.base = frames[-1]
        self.emit("  基线帧: %s" % self.base.hex(" "))
        self.emit(
            "  模拟通道中心: %s" % "  ".join("%s=%d" % (SIM[i], self.base[i]) for i in SIM)
        )
        self.emit("  说明: 按钮变化按 [3][4] 位图判定，模拟量按 [6..9] 判定")

    def buttons_changed(self, d):
        assert self.base is not None
        return any(d[i] != self.base[i] for i in range(32) if i not in SIM)

    def analog_off(self, d, thr=20):
        assert self.base is not None
        return any(abs(d[i] - self.base[i]) > thr for i in SIM)

    # ── 按钮模式 ──
    def press(self, name):
        self.emit("")
        self.emit("▶ 步骤 %s" % name)
        self.emit("  请按下 [%s]，保持约 0.5 秒，然后松开（按回车跳过）" % name)
        pressed = None
        while pressed is None:
            if self.skip_or_key():
                self.emit("  已跳过")
                return
            d = self.read_one(0.2)
            assert self.base is not None
            if d is not None and self.buttons_changed(d):
                pressed = d
                changed = [i for i in range(32) if d[i] != self.base[i]]
                self.emit("  检测到按下: %s" % d.hex(" "))
                self.emit("  变化字节: %s" % " ".join("%d=%02x" % (i, d[i]) for i in changed))
                self.emit("  请松开……")
        while True:
            if self.skip_or_key():
                self.emit("  已跳过（未等释放）")
                return
            d = self.read_one(0.2)
            if d is not None and not self.buttons_changed(d):
                self.emit("  已松开 ✓ 释放帧: %s" % d.hex(" "))
                return

    # ── 摇杆保持模式 ──
    def hold(self, name):
        self.emit("")
        self.emit("▶ 步骤 %s" % name)
        self.emit("  请将 [%s] 推到底并保持不动，不要松（按回车跳过）" % name)
        while True:
            if self.skip_or_key():
                self.emit("  已跳过")
                return
            d = self.read_one(0.2)
            if d is not None and self.analog_off(d):
                self.emit("  检测到偏移，稳定中……")
                break
        t0 = time.time()
        while time.time() - t0 < 0.3:
            self.read_one(0.05)
        mins = {i: 255 for i in SIM}
        maxs = {i: 0 for i in SIM}
        t0 = time.time()
        while time.time() - t0 < 1.5:
            d = self.read_one(0.05)
            if d is not None:
                for i in SIM:
                    mins[i] = min(mins[i], d[i])
                    maxs[i] = max(maxs[i], d[i])
        self.emit("  采样完成，请松开 [%s]（按回车跳过等待）" % name)
        self.emit(
            "  采样范围: %s"
            % "  ".join("%s=%d..%d" % (SIM[i], mins[i], maxs[i]) for i in SIM)
        )
        while True:
            if self.skip_or_key():
                return
            d = self.read_one(0.2)
            if d is not None and not self.analog_off(d, thr=15):
                self.emit("  已回中 ✓")
                return

    # ── 扳机模式：模拟量或按钮位先到为准 ──
    def trigger(self, name):
        self.emit("")
        self.emit("▶ 步骤 %s" % name)
        self.emit(
            "  请将 [%s] 按到底并保持 1 秒：若它是模拟扳机请保持；若是开关请按住（按回车跳过）" % name
        )
        while True:
            if self.skip_or_key():
                self.emit("  已跳过")
                return
            d = self.read_one(0.2)
            if d is None:
                continue
            assert self.base is not None
            if self.analog_off(d):
                self.emit("  检测到模拟量变化:")
                for i in SIM:
                    if abs(d[i] - self.base[i]) > 20:
                        self.emit("    %s: 基线 %d → %d" % (SIM[i], self.base[i], d[i]))
                self.emit("  采样范围: %s" % self._hold_sample())
                self.emit("  请松开（按回车跳过等待）")
                while True:
                    if self.skip_or_key():
                        return
                    d = self.read_one(0.2)
                    if d is not None and not self.analog_off(d, thr=15):
                        self.emit("  已回中 ✓")
                        return
            elif self.buttons_changed(d):
                changed = [i for i in range(32) if d[i] != self.base[i]]
                self.emit("  检测到按钮位变化: %s" % " ".join("%d=%02x" % (i, d[i]) for i in changed))
                self.emit("  请松开……")
                while True:
                    d = self.read_one(0.2)
                    if d is not None and not self.buttons_changed(d):
                        self.emit("  已松开 ✓")
                        return

    def _hold_sample(self, dur=1.5):
        t0 = time.time()
        while time.time() - t0 < 0.3:
            self.read_one(0.05)
        mins = {i: 255 for i in SIM}
        maxs = {i: 0 for i in SIM}
        t0 = time.time()
        while time.time() - t0 < dur:
            d = self.read_one(0.05)
            if d is not None:
                for i in SIM:
                    mins[i] = min(mins[i], d[i])
                    maxs[i] = max(maxs[i], d[i])
        return "  ".join("%s=%d..%d" % (SIM[i], mins[i], maxs[i]) for i in SIM)


def main():
    path = find_hidraw()
    if not path:
        sys.exit("未找到 ZD 接收器，请先插好再运行")
    stamp = time.strftime("%Y%m%d_%H%M%S")
    outfile = "/tmp/zd_map_%s.txt" % stamp
    try:
        out = open(outfile, "w")
    except OSError as e:
        sys.exit("无法写入 %s: %s" % (outfile, e))
    cap = Capture(path, out)
    print("ZD 节点: %s\n输出文件: %s\n开始按屏幕提示操作，回车跳过当前项，Ctrl-C 提前结束。\n" % (path, outfile))

    cap.emit("# ZD 2345:e023 键位捕获  %s" % time.strftime("%Y-%m-%d %H:%M:%S"))
    cap.emit("# hidraw: %s" % path)
    cap.capture_baseline()

    try:
        # 按钮组
        for b in ["A", "B", "X", "Y"]:
            cap.press(b)
        for b in ["LB", "RB", "L3(左摇杆按下)", "R3(右摇杆按下)"]:
            cap.press(b)
        for b in ["BACK/Select", "START", "HOME/Guide"]:
            cap.press(b)
        # 十字键
        for b in ["十字键 UP", "十字键 DOWN", "十字键 LEFT", "十字键 RIGHT"]:
            cap.press(b)
        # 左摇杆
        for d in ["左摇杆 左", "左摇杆 右", "左摇杆 上", "左摇杆 下"]:
            cap.hold(d)
        # 右摇杆
        for d in ["右摇杆 左", "右摇杆 右", "右摇杆 上", "右摇杆 下"]:
            cap.hold(d)
        # 扳机
        cap.trigger("LT(左扳机)")
        cap.trigger("RT(右扳机)")
        # 特殊键自由模式
        cap.emit("")
        cap.emit("▶ 步骤 自由模式")
        cap.emit(
            "  若手柄还有 ST/M1/M2/LM/RM 等特殊键：请逐个按一下（每个保持 0.5s 松开）。"
            "完成后按回车结束（若没有就回车）"
        )
        while True:
            if cap.skip_or_key():
                break
            d = cap.read_one(0.2)
            assert cap.base is not None
            if d is not None and cap.buttons_changed(d):
                changed = [i for i in range(32) if d[i] != cap.base[i]]
                cap.emit(
                    "  特殊键变化: %s" % " ".join("%d=%02x" % (i, d[i]) for i in changed)
                )
                cap.emit("  请松开……")
                while True:
                    if cap.skip_or_key():
                        break
                    d = cap.read_one(0.2)
                    if d is not None and not cap.buttons_changed(d):
                        cap.emit("  已松开 ✓")
                        break
    except KeyboardInterrupt:
        cap.emit("")
        cap.emit("== 用户中断 ==")

    cap.report_non32()
    cap.emit("")
    cap.emit("== 捕获结束，共 %d 帧，报告: %s ==" % (cap.total, outfile))
    out.close()
    print("\n完成！报告文件: %s\n把它发出来即可。" % outfile)


if __name__ == "__main__":
    main()
