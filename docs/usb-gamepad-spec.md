# ZD 手柄（2345 接收器）技术规格

> 精简技术规格，供实现者使用。完整探索过程见 [usb-gamepad-exploration.md](usb-gamepad-exploration.md)。
> 实测环境：Linux 主机 + ESP32-S3（USB-OTG host）+ ZD 2.4G 手柄/接收器。

## 设备标识与模式

| VID:PID | 灯色 | 模式 | 说明 |
|---|---|---|---|
| `2345:e023` | 蓝 | Xinput 风格 | **默认模式**，每次重连回蓝灯 |
| `2345:e024` | 白 | DInput | 需 START+BACK 长按 5s 切换，重连失效 |

接收器插入任意主机（电脑 / ESP32）呈现**相同的描述符与 report**，数据与主机无关。

## USB 接口结构（e023 蓝灯实测）

**只有 2 个接口**（不是 3 个）：

| 接口 | 类 | 端点 | 角色 |
|---|---|---|---|
| 0 | Vendor Specific (0xFF) | EP 0x81 IN / 0x02 OUT（64B 中断） | **完整数据源**：模拟扳机 + 摇杆 + 按钮 |
| 1 | HID (0x03) | EP 0x82 IN / 0x01 OUT | 精简视图：Report ID 4 = 32B 按钮+摇杆（扳机压成数字） |

接口 1 的 report descriptor 声明 4 个 report：ID 1 键盘（16B）、ID 2 鼠标、**ID 4 = 32B 厂商 report**、ID 5 输出（11B）。

## 接口 0 report（64 字节）— 推荐数据源

```
[0]=00 [1]=14                    头（0x14 = 20，xpad 按 Xbox360 手柄识别）
[2] 十字+系统键：↑01 ↓02 ←04 →08 START=10 BACK=20 L3=40 R3=80
[3] LB=01 RB=02 HOME=04 ?=08 A=10 B=20 X=40 Y=80
[4]=LT  [5]=RT                   8-bit 模拟扳机（0-255，松=0）
[6..13]                           第二份摇杆（16bit 区，值域混乱，xpad 用，ESP32 弃用）
[14]=04 [15]=0a [16]=8a          常量头
[17..19]=00                       常量
[20]=LX [21]=LY [22]=RX [23]=RY   8-bit 摇杆（中心 ~128，漂移 ±1-2）
[24..63]=00                       全零（无陀螺仪数据）
```

**按钮位图**（u16 LE：低字节 = report[3]，高字节 = report[2]）：

```
低字节 [3]   高字节 [2]
bit0  LB     bit8  十字↑
bit1  RB     bit9  十字↓
bit2  HOME   bit10 十字←
bit3  (未用) bit11 十字→
bit4  A      bit12 START
bit5  B      bit13 BACK
bit6  X      bit14 L3
bit7  Y      bit15 R3
```

## 接口 1 report（32 字节，Report ID 0x04）— 精简视图

```
[0]=04 报告ID  [1]=0a [2]=8a（常量头）
[3][4][5] = 24-bit 按钮位图
[6]=LX [7]=LY [8]=RX [9]=RY（8bit，中心~128）
[10..31]=00
```

接口 1 按钮位图（与接口 0 布局不同）：B=0x01 A=0x02 Y=0x04 X=0x08 LB=0x10 LT=0x20 L3=0x40 RB=0x80 | RT=0x100 R3=0x200 BACK=0x400 START=0x800 | 十字=0x1000..0x8000 | HOME=0x100000。
**LT/RT 在此视图为数字开关**（位图位），模拟量只有接口 0 有。

## 已知行为

- **扳机**：接口 0 为 8-bit 模拟量（0-255）。xpad 映射后为 16-bit（-32767..+32767），半按可见中间值。
- **LM/RM 背键**：直接映射 LT/RT（数字量、无量程），非独立按键位。
- **陀螺仪**：接口 0 无陀螺仪数据（说明书提及但实测未发现；可能需手柄端开启或仅其他模式）。
- **中心漂移**：摇杆/扳机中心值随连接漂移（实测 LX=128 LY=118~135 等），解析时应动态取基线或留死区。

## 电脑端读取

| 方式 | 工具 | 说明 |
|---|---|---|
| 接口 1 | `/dev/hidraw*`（`tools/hid_capture.py` / `tools/zd_capture.py`） | 免 root（udev uaccess）；拿到 32B 精简 report |
| 接口 0（xpad 视角） | `/dev/input/js1`（`tools/js_read.py`） | 内核 xpad 自动接管，轴名即语义（LT/RT 模拟） |
| 接口 0（原始） | pyusb（`tools/iface0_capture.py` / `tools/iface0_guide.py`） | 需 root 或装 `tools/91-zd-controller.rules` 后重插；先解绑 xpad |

解绑 xpad：`sudo sh -c 'echo 1-<bus>:1.0 > /sys/bus/usb/drivers/xpad/unbind'`（设备路径见 `lsusb -t`）。

## ESP32 读取要点（已真机验证）

1. **必须用接口 0**（`VendorInHost` 自写，绑定 EP 0x81 中断 IN，绕过 `HidHost::find_hid` 只认 HID 类的限制）。
2. **必须校验 VID == 0x2345** —— hub 等设备也有 vendor 接口（如 VIA 2109:2211 状态 EP），不校验会把 hub 误当接收器，read 永远无数据且接收器不被枚举。
3. **read 超时要 break**（5s 无报告 = 设备拔出/绑定错误），否则挂起卡死。
4. 经 hub 连接时：枚举先见 hub → VID 校验拒绝 → 走 hub 注册 → 下游枚举接收器。
5. 转向建议：`angle = (rt - lt) * max_deg / 255`（LT 左转、RT 右转，量程决定幅度）。
