# 灯光系统 — 设计文档

> 来源：[探索记录 2026-07-25](./2026-07-25.exploration.md)
> 分析日期：2026-07-25

## 目标

利用板载 WS2812 + PCF8575 I2C IO 扩展驱动外接 LED，实现车载状态指示与车灯系统。
**核心价值：不用盯串口就知道系统状态。**

---

## 三层子系统

灯光系统按关注点拆为三个独立的子系统：

| 子系统 | 目的 | 硬件 | 触发源 |
|--------|------|------|--------|
| **状态 / 调试灯** | 让开发者知道系统在干什么 | 单颗 WS2812（板载 G48 → 未来外接） | 系统事件驱动 |
| **车灯** | 让车对外"说话" | PCF8575 I2C + 普通 LED（多颗） | 操控行为驱动 |
| **氛围灯** | 纯视觉效果 | 暂无硬件（未来 WS2812 灯带） | 持续运行 / 车灯联动 |

---

## 硬件

### 已有资源

| 资源 | 位置 | 用途 |
|------|------|------|
| WS2812 ×1 | 开发板板载 **G48** | 调试时状态指示 |
| PCF8575 ×1 | I2C（地址 0x20-0x27） | 16 路 IO 扩展，驱动车灯 |
| MCP23017 ×1 | I2C（备用） | 推挽输出，预留做输入扩展或后续功能 |

### 引脚分配

| 引脚 | 功能 | 接口 |
|------|------|------|
| **G48** | 板载 WS2812 | RMT |
| 待定 SDA | PCF8575（车灯） | I2C |
| 待定 SCL | PCF8575（车灯） | I2C |

### 硬件选型决策

#### PCF8575 vs MCP23017

| | PCF8575 | MCP23017 |
|---|---------|----------|
| 输出类型 | 准双向（sink 25mA） | 推挽，25mA source/sink |
| 驱动 LED | ✅ sink 方式（LED 阳极接 VCC） | ✅ source 或 sink 均可 |
| 初始化 | 零配置，上电即用 | 需写 IODIR 寄存器设方向 |
| 寄存器 | 极简：写 2 字节 | 需配置方向寄存器 |
| 结论 | **选它驱动车灯（代码更简单）** | 留作输入/复杂扩展 |

#### PCF8575 概要

- 16 路准双向 IO，写 2 字节即控制全部 pin
- 输出 0：sink 电流 ~25mA，LED 亮（阳极接 VCC + 限流电阻）
- 输出 1：弱上拉到 VCC（~100µA source），LED 灭
- 上电默认全部高电平（LED 灭），无需初始化寄存器
- 2 线 I2C 走全场，16 颗 LED 不占额外 GPIO

### GPIO 预算

| 状态 | 数量 | 说明 |
|------|------|------|
| 当前已占用 | 12 | motor/servo/PS2 |
| 已分配 | 4 | 电池 ADC(G8) + WS2812(G48) + 喇叭 SD(G17, 未来) |
| Strapping | 20 | 不可用 |
| **空闲** | **11** | G15,16,18,21,38,41,42,43,44,47 + 喇叭 I2S 待定 |

> G48 已锁定给板载 WS2812（不可改）。G38 仍可用。

### 扩展规划

- WS2812 驱动代码与物理灯解耦——改一个 GPIO 即可从板载切到外接灯带
- PCF8575 的 16 路仅用车灯 4-6 路，剩余 10+ 路可用于后续功能
- MCP23017 可作为 PCF8575 的补充（推挽输出、独立方向控制），或做输入扩展
- 氛围灯：后续 WS2812 灯带（8 颗覆盖车头/车尾/两侧）

---

## 软件设计

### 目录结构

```
src/
  lighting/
    mod.rs        ← pub Lighting struct，统一入口
    ws2812.rs     ← WS2812 底层驱动（RMT 时序），被 status 和 ambient 复用
    status.rs     ← 子系统1：调试灯状态机
    vehicle.rs    ← 子系统2：车灯（转向 / 刹车 / 大灯）
    ambient.rs    ← 子系统3：氛围灯（stub）
  battery.rs     → is_low()        → 红慢闪
  control.rs     → DriveMode       → 绿/蓝常亮
  ps2.rs         → 连接/断连事件    → 黄快闪
```

`Lighting` struct 作为统一入口，main 只跟它对话：

```rust
// main.rs
let mut lighting = lighting::Lighting::new(
    peripherals.GPIO48,            // WS2812
    i2c,                           // PCF8575
);

loop {
    // ... motors, steering ...

    lighting.status.update(&status_input);
    lighting.vehicle.update(&vehicle_input);
    // lighting.ambient.update(&ambient_input); // future
}
```

### 模块职责边界

| 层 | 职责 | 依赖 |
|---|------|------|
| `ws2812.rs` | RMT 时序 + RGB 写入 | `esp-hal::rmt`（或 SPI bit-bang 兜底） |
| `status.rs` | 调试灯状态机 + 动画 | `ws2812` |
| `vehicle.rs` | 转向灯 / 刹车灯 / 大灯逻辑 | `esp-hal::i2c` → PCF8575 |
| `ambient.rs` | 氛围灯（stub，声明 trait） | `ws2812`（未来） |
| `mod.rs` | 组合三个子系统，暴露 `update` | 以上全部 |

---

## 子系统1：状态 / 调试灯

### 颜色映射

| 颜色 | 含义 | 数据来源 |
|------|------|----------|
| 绿常亮 | 舵机模式 / 就绪 | `DriveMode::Servo` |
| 蓝常亮 | 差速模式 | `DriveMode::Diff` |
| 红慢闪 | 低电量 | `BatteryMonitor::is_low()` |
| 黄快闪 | PS2 断连 | `Ps2Event::LostAnalog` |
| 白渐亮 | 启动自检完成 | 初始化阶段 |

### WS2812 驱动方案

#### 路径 A：RMT（推荐）

`esp-hal` 有 `rmt` 模块，可生成 WS2812 精确时序（T0H=0.35µs, T0L=0.8µs, T1H=0.7µs, T1L=0.6µs, RESET>50µs）。ESP32-S3 有 8 个 RMT 通道，占用一个。

```rust
// 预期 API 形态（待验证）
let rmt = Rmt::new(peripherals.RMT, 80.MHz())?;
let ws2812 = Ws2812::new(rmt, peripherals.GPIO48);
ws2812.write(Rgb { r: 0, g: 255, b: 0 }); // 绿
```

#### 路径 B：SPI bit-bang（兜底）

用 SPI MOSI 模拟时序（每 bit → 特定 bit pattern）。不需要额外外设，但代码较 hacky。单颗 WS2812 完全可行。

**决策：优先 RMT，SPI 兜底。**

#### 性能

- 单颗 WS2812 写入：~30µs × 24 bits + 50µs reset ≈ **0.8ms**
- 在 33ms tick 中完全不影响实时性

### 状态机（修订版）

#### 分层优先级模型

原设计用"打断"模型，但**同时发生多个异常时行为未定义**。

改为分层覆盖：

```
Layer 0 (基底):  Off / 白渐亮(自检) / 模式色(绿/蓝)
Layer 1 (覆盖):  红慢闪(低电量)
Layer 2 (覆盖):  黄快闪(PS2 断连)
```

**竞争策略**：PS2 断连（Layer 2）覆盖低电量（Layer 1）——因为断连是紧急事件，用户需要立刻知道无法操控。低电量是持续性告警，可以稍微延后。

如果用户认为低电量更重要，可以改为低电量 > 断连。只需改一个 match 顺序。

#### 状态转换表

| 当前 | 事件 | 行为 |
|------|------|------|
| Off | 启动 | 白渐亮动画（~500ms） |
| 白渐亮 | 动画完成 | 根据 mode 显示绿/蓝常亮 |
| 绿/蓝常亮 | 模式切换 | 换色（无动画，直接切） |
| 任意 | 低电量 | 红慢闪（500ms on / 500ms off） |
| 任意 | PS2 断连 | 黄快闪（150ms on / 150ms off） |
| 红慢闪 | 低电量恢复 | 回到模式色常亮 |
| 黄快闪 | PS2 恢复 (`RecoveredAnalog`) | 回到模式色常亮（若低电量则回红慢闪） |
| 任意 | 低电量 + PS2 断连 | 黄快闪（Layer 2 > Layer 1） |

#### 渐变动画

```rust
enum Anim {
    Solid(Rgb),
    Fade { from: Rgb, to: Rgb, step: u8, max_steps: u8 },
    Blink { color: Rgb, on_ticks: u8, off_ticks: u8, tick: u8, on: bool },
}
```

---

## 子系统2：车灯

### 硬件

PCF8575 I2C IO 扩展器，2 线（SDA/SCL）→ 16 路准双向 IO。

LED 接法：阳极 → 限流电阻 → VCC，阴极 → PCF8575 pin。pin 写 0 亮，写 1 灭。

```rust
// 写 2 字节 = 16 pin 状态，无需初始化寄存器
// bit 0 (P00): 刹车灯    bit 8  (P10): 右转向灯
// bit 1 (P01): 左转向灯  bit 9+ (P11-P17): 预留
// bit 2 (P02): 大灯
let lo = brake << 0 | left_signal << 1 | headlight << 2;
let hi = right_signal << 0;  // P10
// 注意：0 = 亮（sink），1 = 灭（弱上拉）。逻辑取反后写入
i2c.write(addr, &[!lo, !hi]);
```

### GPIO 映射

| PCF8575 | 功能 | 灯色 |
|---------|------|------|
| P00 | 刹车灯 | 红 |
| P01 | 左转向灯 | 黄 |
| P02 | 前大灯 | 白 |
| P10 | 右转向灯 | 黄 |
| P03-P07,P11-P17 | 预留 | — |

> 只有开/关，无 PWM 调光。如需亮度控制，后续可换 PCA9685（16 路 12-bit PWM），I2C 地址不同（0x40），代码接口兼容。

### 刹车灯

检测 LY > 128 + deadzone（后退/刹车），直接输出：

```rust
let braking = state.ly() > 128 + cfg.ly_deadzone;
```

无时序，纯 on/off。

### 转向灯

自动检测 RX 方向 + 回正延时：

```rust
struct TurnSignal {
    active: Option<Direction>,  // None / Left / Right
    hold_ticks: u8,             // 回正后倒计时 ≈ 2s
    blink_tick: u8,             // 独立闪烁计数器
}

// 每 tick:
//   RX > deadzone  → active = Right, hold_ticks = 60 (~2s)
//   RX < -deadzone → active = Left,  hold_ticks = 60
//   RX 回中        → hold_ticks -= 1，归零后 active = None
//   灯亮灭节律      → blink_tick % 15 < 7 (≈500ms 翻转)
```

60 ticks × 33ms ≈ 2 秒。参数可调。

触发源：纯自动（RX 摇杆方向），无需手动按键。

### 前大灯

GPIO on/off，按键切换（如 SELECT），5 行代码。

---

## 子系统3：氛围灯

> 暂无硬件，先设计 trait 占位。

### 用途

- WS2812 灯带（8+ 颗）
- 流水灯、呼吸灯、彩虹渐变等视觉效果
- 可与车灯联动（刹车 → 红色流水；转向 → 黄色指向）

### 接口占位

```rust
/// 氛围灯抽象 trait —— 未来硬件替换不影响上层逻辑。
pub trait AmbientLight {
    fn update(&mut self, input: &AmbientInput);
}

pub struct AmbientInput {
    pub braking: bool,
    pub turning: Option<Direction>,
}
```

当前提供一个空实现（no-op），后续换 WS2812 灯带时只需替换 `AmbientLight` 实现。

---

## 与主循环集成

每个 33ms tick 末尾，在 `Timer::after` 之前统一更新：

```rust
// 在每个 tick 结束前
let status_input = lighting::StatusInput {
    mode,
    battery_is_low: battery_monitor.is_low(),
    ps2_connected: matches!(last_event, Ps2Event::Analog(_)),
};

let vehicle_input = lighting::VehicleInput {
    braking: state.ly() > 128 + cfg.ly_deadzone,
    rx: state.rx(),
    headlight: state.pressed(Button::Select), // 待定
};

lighting.status.update(&status_input);
lighting.vehicle.update(&vehicle_input);
```

`update()` 内部负责：

1. 根据输入更新状态机
2. 计算当前帧的 WS2812 RGB
3. 通过 RMT 写入（~0.8ms 阻塞）
4. 通过 I2C 写入 PCF8575 GPIO

---

## 待调研

- [ ] `esp-hal::rmt` 在 v1.1.0 + `unstable` 功能下的 API 形态和 WS2812 兼容性
- [ ] `esp-hal::i2c` API 形态 + PCF8575 在 no_std 下的寄存器操作
- [ ] G48 板载 WS2812 — 已确认（通断测试）
- [ ] 外接灯带时供电 — 板载 3V3 不够驱动多颗 WS2812，需从电池取电
- [ ] SPI bit-bang 兜底方案评估（当 RMT 不可用时）
- [ ] PCF8575 sink 驱动 LED 限流电阻取值（VCC 电压 → 亮度 → 电阻计算）
