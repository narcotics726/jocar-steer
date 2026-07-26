# 灯光系统 — 架构讨论 2026-07-26

> 来源：PCF8575 验证通过后，讨论 vehicle 子系统设计及整体控制流架构。
> 关联：[lighting.md](./lighting.md) — 原始灯光系统设计

---

## PCF8575 验证

| 项目 | 结论 |
|------|------|
| I2C 总线 | SDA=G17, SCL=G18, I2C0 |
| 地址 | 0x20（A0=A1=A2=GND） |
| API | `esp_hal::i2c::master::I2c`，无需额外 feature |
| 写操作 | `i2c.write(0x20, &[low_byte, high_byte])`，2 字节控制 16 路 |
| 驱动逻辑 | 写 0 = 亮（灌电流），写 1 = 灭（高阻）。模块内部用正向逻辑，输出时取反 |
| 测试 bin | `src/bin/pcf8575-test.rs` |

---

## Vehicle 子系统设计结论

### 架构

- **纯状态机**：不持有 I2C 硬件，只根据输入返回 `u16` 端口值（已取反，可直接写 I2C）
- 调用者负责 I2C 写入，以及 dirty check 去重

### 引脚映射

可配置 `VehiclePinConfig`，用 bit mask 指定每路功能：

```rust
pub struct VehiclePinConfig {
    pub brake:        u16,  // 默认 1 << 0   (P00)
    pub left_signal:  u16,  // 默认 1 << 1   (P01)
    pub headlight:    u16,  // 默认 1 << 2   (P02)
    pub right_signal: u16,  // 默认 1 << 8   (P10)
}
```

改接线只需改 config，逻辑代码不感知物理 pin 位置。

### 职责边界

| 层 | 职责 |
|----|------|
| **InputProcessor**（见下） | RX 转方向 + 死区、按键去抖 + edge detection |
| **vehicle 模块** | 接收转向请求 `left/right: bool`，只管闪烁节律（500ms 周期）+ 释放后 hold 2s。刹车/大灯直接映射 |

vehicle 不关心转向请求来自自动检测还是手动按键——对它是透明的。

### 转向灯

- **触发**：自动（RX 摇杆方向）+ 手动（按键 toggle）
- **取消**：手动再按同方向取消；自动/手动均可被对方覆盖
- **hold**：请求消失后持续闪烁 2s（摇杆回正缓冲）
- **闪烁**：500ms 周期（250ms on / 250ms off），独立于激活/取消逻辑
- **刹车 + 转向**：同时亮（物理上不同 LED，无冲突）

### 大灯

- 简单 toggle，由 Select 按键触发
- 调用者去抖后传入

---

## 架构演进：InputProcessor + ControlEvent

### 问题

当前 main.rs 中原始摇杆值（`state.ly()`, `state.rx()`）直接散落入各子系统。加车灯后会在主循环散落更多 ad-hoc 判断（`ly>128`, `rx>deadzone` 等），重复且易出错。

### 方案

在 PS2 原始输入和子系统之间加一层 **InputProcessor**，单点产生统一的 **ControlEvent**，广播给所有子系统消费。

```
Ps2Event::Analog(state)
        │
   ┌────▼────────────────────┐
   │  InputProcessor         │  单点，纯函数
   │  state → ControlEvent   │
   └────┬────────────────────┘
        │ ControlEvent (广播)
   ┌────┼────────┬───────────┐
   ▼    ▼        ▼           ▼
 Motors Steering Lights   (future)
```

### ControlEvent 形态

```rust
pub struct ControlEvent {
    /// 油门：-max_duty..+max_duty，正=前进，负=后退
    pub throttle: i32,
    /// 转向：-max_deg..+max_deg，正=右，负=左
    pub steer: i32,
    /// 本 tick 新按下的按键（edge-triggered）
    pub buttons_pressed: EnumSet<Button>,
    /// 当前持续按压的按键
    pub buttons_held: EnumSet<Button>,
}
```

### 子系统消费方式

| 子系统 | 消费字段 | 行为 |
|--------|---------|------|
| Motors | `throttle` | 正转/反转/停 |
| Steering | `steer` | 舵机角度 |
| Vehicle lights | `throttle`, `steer`, `buttons_pressed` | 刹车 = `throttle < 0`; 转向 = `steer` 方向; 大灯 = Select edge |
| Status light | `buttons_pressed` + 外部输入 | 模式色 / 告警闪烁 |
| Mode switch | `buttons_pressed` ∩ {L3,R3} | 切换 DriveMode |

### 对 Vehicle 接口的影响

Vehicle 不再需要独立的 `VehicleInput` struct——直接从 `ControlEvent` 取数据：

```rust
impl VehicleLights {
    pub fn apply(&mut self, event: &ControlEvent, dt_ms: u16) -> u16;
}
```

`InputProcessor` 已解决死区、摇杆范围、按键去抖，Vehicle 只消费语义化的驾驶意图。

---

## 实施顺序建议

1. **InputProcessor** — 架构枢纽，~30 行，先建
2. **vehicle.rs** — 纯状态机，对接 `ControlEvent`
3. **主循环重构** — 从散落的 stick 值调用改为统一 dispatch
4. **status 灯接入** — 已有 `Ws2812StatIndicator`，改为接收 `ControlEvent` + 电池/PS2 状态
