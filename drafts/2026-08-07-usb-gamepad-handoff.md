# USB 手柄接收器 — 会话交接文档（2026-08-07）

> 新会话从这里继续。读这个文件 + `src/bin/usb-gamepad-test.rs` + `src/bin/usb-gamepad-map-test.rs` 即可恢复上下文。
>
> ⚠ **2026-08-15 更新**：整车已定型（`usb-gamepad-car` 单电机，2S 7.4V 电池）。当前硬件/接线/控制状态以 `AGENTS.md` 和 `docs/wiring.md` 为准；本文件的接线信息（如舵机 GPIO13、写死端口）属于当时测试阶段，仅作历史。

## 一、已完成 ✅

| 项目 | 状态 | 说明 |
|---|---|---|
| TTL 刷写 + defmt 日志 | ✅ | runner = `espflash flash --monitor -L defmt`（.cargo/config.toml）；端口自动探测（旧版写死 `--port /dev/ttyUSB0`，2026-08 已移除）；`[alias] test` 切回 probe-rs |
| 日志通道 | ✅ | rtt → esp-println（defmt over UART0/GPIO43-44），自定义 `#[panic_handler]` |
| esp-hal 升级 | ✅ | git rev `667f8f02`（**必须 git，crates.io 1.1.2 没有 USB Host**）；esp-rtos/esp-bootloader-esp-idf/esp-storage 同 rev |
| **USB Host + hub** | ✅ | `Usb::new_fs(USB_FS, GPIO20, GPIO19)` + `embassy_usb_host` + `HubHandler::<_, 8>` 遍历下游（接收器 USB-A 经 hub 进 USB-C OTG 口） |
| **ZD 接收器 DInput 模式读取** | ✅ | e024（白灯）：9 字节标准 HID，摇杆/16键/十字/扳机全部工作 |
| **舵机联动** | ✅ | 左摇杆 X → MG90S（GPIO13, LEDC Timer0 Ch0 @50Hz 12bit）转向正常 |
| flash 脱机日志 | ✅ | NVS 分区前 2KB，单实例 FlashStorage（**必须单实例**，重复 new 会坏 ROM flash 读），boot dump（当前 map-test 版本为保留不擦除） |
| e023 布局逆向 | ⚠️ 进行中 | map-test 已捕获部分数据，日志写满丢失，需改进后重测 |

## 二、关键硬件知识（血的教训）

1. **供电**：板子 LDO 是 **AMS1117-3.3**（需要 ≥4.4V 输入）。3.7V 锂电直供只有 ~4V → 3.3V 掉到 2.8-3.0V → brownout 锁死（RESET 无效，只有彻底断电恢复）。**必须 5V 供电**（升压模块或 5V 灌入 5V/VBUS 引脚）。7.4V 别直接灌板子（电容 6.3V 耐压）——2026-08 实际方案：2S 7.4V 先经 **5V buck** 再进 5VIN，已规避。
2. **物理连接**：TTL 与 OTG 口太近不能同时插；接收器是 USB-A、板子 OTG 是 USB-C → **必须经 hub/转接头**，所以固件必须支持 hub。
3. **ZD 接收器（VID 2345）三态**：
   - **蓝灯 = Xinput**（e017 键盘态 / e023 键盘+鼠标+厂商接口，32 字节厂商 report）— **每次重连默认蓝灯**
   - **白灯 = DInput**（e024，9 字节标准 HID）— 需 START+BACK 5 秒切换
   - 目标：直接解析蓝灯 e023，免去每次手动切白灯

## 三、已解码的数据

### e024（DInput，已验证，9 字节 report）

```
[0..3] X Y Z Rz（0-255，中心 128）   [4] 低半字节 hat（空置=8！不是 15）
[5..6] 16 位按钮位图                 [7] accel [8] brake
```

注意：**hat 空置值是 8**（代码里 `hat < 8` 才算按下）。

### e023（Xinput，**已完整解码**，2026-08-08 交互式捕获确认，32 字节 report）

```
[0]=04 报告ID   [1]=0a  [2]=8a（恒定头）   [3][4][5]=24位按钮位图   [6..9]=LX LY RX RY（8bit，中心~128）   [10..31]=00
```

**按钮位图**（任天堂布局，与 Xbox 反序）：

| 字节 | 位 | 键 | 字节 | 位 | 键 |
|---|---|---|---|---|---|
| [3] | 0 | B | [4] | 0 | RT（**数字**） |
| [3] | 1 | A | [4] | 1 | R3 |
| [3] | 2 | Y | [4] | 2 | BACK |
| [3] | 3 | X | [4] | 3 | START |
| [3] | 4 | LB | [4] | 4 | 十字← |
| [3] | 5 | LT（**数字**） | [4] | 5 | 十字→ |
| [3] | 6 | L3 | [4] | 6 | 十字↑ |
| [3] | 7 | RB | [4] | 7 | 十字↓ |
| [5] | 0-3 | 特殊键 S1-S4 | [5] | 4 | HOME |
| [5] | 5 | 特殊键 S5 | [5] | 6-7 | 未用 |

**模拟通道**：`[6]=LX`（左0/右255）`[7]=LY`（上~0/下255）`[8]=RX`（左0/右255）`[9]=RY`（上~0/下255）。
中心值会漂移（实测 LX=128 LY=118/127 RX=128/130 RY=130/129）→ **解析必须动态取基线**，不能硬编码。

**要点**：LT/RT 是数字开关（位图位），与 e024 的模拟扳机不同；十字键走位图，无 hat 字节；特殊键 S1-S5 标签待确认（ST/M1/M2/LM/RM？）。

### 接口 0（Vendor Specific 类，EP 0x81，**2026-08-08 完整解码**）

**这是 Xinput 模式的完整数据源**：模拟扳机 + 双摇杆 + 全部按钮，一个接口全有（接口 1 只是它的精简视图）。64 字节 report：

```
[0]=00 [1]=14                    头（xpad 当 Xbox360 手柄识别，Linux 上自动映射为 js1）
[2]=十字+系统键: ↑01 ↓02 ←04 →08 START10 BACK20 L3=40 R3=80
[3]=LB=01 RB=02 HOME=04 ?=08 A=10 B=20 X=40 Y=80
[4]=LT 8bit 模拟   [5]=RT 8bit 模拟
[6..13]=第二份摇杆（16bit，xpad 用，值域混乱，ESP32 弃用）
[14]=04 [15]=0a [16]=8a         常量头
[17..19]=00                     常量
[20]=LX [21]=LY [22]=RX [23]=RY   8bit 摇杆（中心 ~128，与接口 1 [6..9] 同步）
[24..63]=00                     无陀螺仪数据
```

**按钮位映射**（u16：低字节=report[3]，高字节=report[2]）：
`LB=0x01 RB=0x02 HOME=0x04 A=0x10 B=0x20 X=0x40 Y=0x80 | D_U=0x100 D_D=0x200 D_L=0x400 D_R=0x800 START=0x1000 BACK=0x2000 L3=0x4000 R3=0x8000`

**验证方法（电脑上，无需 ESP32）**：xpad 自动接管接口 0 → `/dev/input/js1`（axis: 0=LX 1=LY 2=LT 3=RX 4=RY 5=RT）→ `tools/js_read.py` 实时看轴。扳机半按可见中间值（实测 -32767→0→+32767 连续）。解绑 xpad（`echo 1-3.4:1.0 > /sys/bus/usb/drivers/xpad/unbind`）后用 `tools/iface0_capture.py`/`tools/iface0_guide.py`（pyusb 直读）看原始 report。

**陀螺仪**：接口 0 无陀螺仪数据（转动手柄 [24..63] 全 0）。说明书提到陀螺仪，可能需手柄端开启或仅其他模式有 —— 未深挖，小车用不到。

## 四、进度（2026-08-08 更新）

**已完成**：

- ✅ **电脑端抓取管道**（不再依赖 ESP32 段间拔插）：接收器是标准 USB HID，Linux 上 `/dev/hidraw`（接口 1，Report ID 1/2/4/5 混合，32B 厂商 report 是 ID 4）无需 root 直接可读。接口 0 是 Vendor Specific 类（EP 0x81/0x02），内核不绑 hidraw，libusb 才能读。
  - `tools/hid_capture.py` — 实时抓取 + 变化字节标注
  - `tools/zd_capture.py` — 交互式按键向导（逐个提示，摇杆保持采样，回车跳过，Ctrl-C 结束），输出 `/tmp/zd_map_*.txt`
- ✅ **e023 完整解码**（见上文三节，2026-08-08 交互捕获确认）：24 位按钮位图 + [6..9] 四模拟通道，LT/RT 为数字扳机，ABXY 为任天堂布局，十字键走位图无 hat。特殊键 ST/M1/M2/LM/RM（后四个背键，标签顺序未确认，可能与标准键重复）。
- ✅ **usb-gamepad-test.rs 双模式解析**：e023（蓝灯默认）+ e024（白灯）兼容，按钮位图升 u32，LT/RT 数字扳机映射进 accel/brake。编译通过，**待真机验证舵机转向**。

**下一步**：

1. ~~真机验证：刷 `usb-gamepad-test`，蓝灯模式直接推左摇杆 X 看舵机（无需切白灯）~~ ✅ **已完成**（接口 1 舵机验证，后升级到接口 0）
2. ~~抽 lib~~ ⬜（`src/usb_gamepad.rs`：枚举 + hub + 接口 0 解析 → 归一化 InputFrame；`src/flash_log.rs` 收编两 bin 重复日志代码）
3. ~~集成 main.rs~~ ⬜（构建期 feature：`default=["ps2"]` + `usb-gamepad`，cfg-gated 输入源）
4. ~~完整控制固件~~ ⬜（转向 + 马达 + 输入源）

**2026-08-08 晚更新（接口 0 真机验证成功）**：

- `usb-gamepad-test.rs` 改用 **VendorInHost**（自写，绑接口 0 EP 0x81，绕过 HidHost 的 find_hid），**转向改为 LT/RT 线性扳机**（`triggers_to_angle`，0-255 → 0..±90°）。真机验证：舵机跟随扳机量程 ✅，按键 LED 分类 ✅。
- **踩坑修复**：① VendorInHost 必须校验 `device_desc.vendor_id == 0x2345` —— 否则 hub（如 VIA 2109:2211）的 vendor 状态 EP 会被误绑定，read 永远无数据且**接收器不被枚举**（实测症状：`iface0 ready` + `no reports for 5s` 死循环 + 拔 hub 卡死）；② read 超时（5s 无报告）要 `break` 而非无限循环。
- **新发现**：手柄 **LM/RM 背键直接映射 LT/RT**（数字量、无量程）—— 不是独立按键位。
- 诊断手段：`pyserial` 直接读 TTL 串口 + 复位触发 boot dump（`flash_log_dump` 打印 NVS 日志），无需 espflash monitor（非交互 stdin 会崩）。

## 五、关键文件

- `.cargo/config.toml` — espflash runner、test alias
- `Cargo.toml` — git 依赖（esp-hal/esp-rtos/esp-bootloader-esp-idf/esp-storage @ 667f8f02 + embassy-usb-host 0.1.0 + embassy-usb-driver 0.2.1）
- `src/bin/usb-gamepad-test.rs` — 主测试：hub + e024 读取 + 舵机联动 + LED 状态机
- `src/bin/usb-gamepad-map-test.rs` — e023 映射工具（需改进）
- flash 日志：NVS 分区（0x9000）前 2KB，`LOG_MAGIC = "JCLG"`，boot 时 dump 到 UART

## 六、测试工作流（无 TTL 时）

```
插 TTL 刷写 → 拔 TTL → hub+接收器插 OTG 测试 → 拔 hub → 插 TTL → 复位读 flash 日志
```

（`cargo run --bin <bin>` 刷写；读日志用 `cargo run` 重刷触发 boot dump，或 espflash monitor 复位）
