# ZD 手柄探索过程回顾

> 简述从"在 ESP32 上抓键位"到"完整解码 + 真机验证"的探索旅程，记录弯路与教训。
> 技术规格见 [usb-gamepad-spec.md](usb-gamepad-spec.md)。

## 起点：原计划是在 ESP32 上抓

最初方案完全在嵌入式侧：

- 改进 `usb-gamepad-map-test.rs`（模拟量死区、日志扩到整个 NVS、分段按键序列）
- 用 ESP32 的 flash 日志记录 untethered 测试（拔 TTL 插 OTG，按键后插回 TTL 读 dump）
- 计划"分区分次"捕获：摇杆段、按钮段、完整序列，每段之间用 esptool 擦除

这段路还没走就绕过去了。

## 转折 1：电脑直读（最大的一次效率跃升）

用户问了一个"很蠢"的问题：**为什么不在电脑上把键位读完？**

对 —— 接收器是标准 USB HID 设备，插电脑和插 ESP32 数据一模一样，而 Linux 上：

- 接口 1 有 `/dev/hidraw` 节点（免 root）
- 抓取是实时的、交互的，不用拔插 TTL
- report descriptor 本身就是布局说明书

于是写了 `tools/hid_capture.py`（实时抓 + 变化标注）和 `tools/zd_capture.py`（交互式按键向导：逐个提示、等你按下松开、回车跳过、摇杆保持采样）。**教训：先问"这个数据谁还能读到"，再决定在哪里实现。**

## 转折 2：接口 1 的 4-report 结构

电脑上先解出接口 1 的 report descriptor，发现它**一个接口声明了 4 种 report**：
键盘（ID 1）、鼠标（ID 2）、**32 字节厂商 report（ID 4）**、输出（ID 5）。

交互向导跑完，32 字节 report 全解码：24-bit 按钮位图 + 4×8bit 摇杆。但有个疑点：**LT/RT 在这个视图里是数字开关**（位图位），而说明书说 Xinput 有线性扳机。

## 转折 3：xpad 接管 —— 模拟扳机浮出水面

检查接口归属时发现：**内核 xpad 驱动把接口 0 当 Xbox360 手柄接管了**（生成 `/dev/input/js1`）。用 `tools/js_read.py` 一测：**RT 是完整 16-bit 模拟量**（-32767 → 0 → +32767 平滑过渡）。

结论：**e023 的完整数据（模拟扳机）在接口 0**，接口 1 只是精简视图。而且 xpad 的接管本身就是线索 —— 设备在模拟 Xbox360 协议。

## 转折 4：接口 0 完整解码（又一个弯路）

解绑 xpad 后用 pyusb 直读接口 0（`tools/iface0_guide.py` 交互引导，这次学乖了：**逐动作交互而非定时抓流** —— 实时抓流 125 帧/秒人类按键根本跟不上）。

最终布局：64 字节 = Xbox360 风格头（00 14）+ [4][5] 模拟扳机 + [2][3] 16-bit 按钮 + [20..23] 8-bit 摇杆 + [6..13] 混乱的 16-bit 副本（xpad 用）。**无陀螺仪数据。**

**教训**：交互式引导（等按下/保持/松开）比定时窗口可靠得多 —— 人类操作节奏不该去适配设备轮询频率。

## 转折 5：ESP32 侧 —— VendorInHost 与 hub 陷阱

ESP32 上用接口 1（HidHost）先验证了舵机转向（蓝灯免切灯 ✅），但既然模拟扳机在接口 0，就写了 `VendorInHost`：自己解析 config descriptor 找接口 0 的中断 IN EP，`alloc_pipe` 绑定，绕过 `HidHost::find_hid` 只认 HID 类的限制。约 70 行。

**踩了三个坑**：

1. **`UsbPipe::read` 不存在** —— 方法是 `request_in`（`read` 是 HidHost 自己的包装）。
2. **hub 误绑定**（最大坑）：用户接的是 hub + 接收器。枚举先见 hub（VIA 2109:2211），而 **hub 的接口 0 也是 vendor class + 中断 IN EP（状态端点）**，VendorInHost 绑定"成功"了 → read 永远无数据（hub 状态 EP 平时静默）→ 接收器从不被枚举。症状：`iface0 ready` + `no reports for 5s` 死循环，拔 hub 卡死。**修复：校验 `device_desc.vendor_id == 0x2345`。**
3. **超时不 break**：read 挂起时 with_timeout 兜底，但超时分支只重置计时器不退出 → 拔设备后死循环。**修复：5s 无报告 break。**

修完后真机验证：**LT/RT 线性扳机驱动舵机**（量程 0-255 → ±90°，按深决定幅度）✅，按键 LED 分类 ✅。还顺带发现 **LM/RM 背键 = LT/RT 的数字映射**。

## 其他弯路

- **cargo test 噪音**：no_std 项目里 host 上跑 `cargo test` 必然报 no-global-allocator（`[alias] test` 被 probe-rs 覆盖），工具链反复误报。**直接删掉测试套件，`[alias] test = "build"`。**
- **espflash monitor 非交互崩溃**：`Failed to initialize input reader`（stdin 不可用）。改用 **pyserial 直接读串口 + DTR/RTS 复位触发 boot dump**，可靠且免 defmt 解码（flash 日志是纯文本）。
- **flash dump 即读即清**：`flash_log_dump` 打印后清除，错过时机就再也读不到测试日志 —— 测试前先 `espflash erase-region 0x9000 0x6000` 拿干净起点，测完及时读。

## 沉淀的工具（tools/）

| 工具 | 用途 |
|---|---|
| `hid_capture.py` | 接口 1 实时抓取 + 变化字节标注 |
| `zd_capture.py` | 接口 1 交互式按键向导（生成映射报告） |
| `js_read.py` | xpad 视角（js1）轴/按钮实时查看 |
| `iface0_capture.py` | 接口 0 原始 report 实时抓取（pyusb） |
| `iface0_guide.py` | 接口 0 交互式字节映射向导 |
| `91-zd-controller.rules` | udev 规则：非 root libusb 访问接口 0 |

## 一句话总结

**先确认数据在哪里、谁能读（电脑比嵌入式强百倍），再决定实现路径；解码要交互式引导而非定时抓流；绑定设备接口必须校验 VID，读不到数据要超时退出而非死等。**
