# 双模控制 — 最终设计文档

**日期**: 2025-07-21

---

## 1. 硬件布局

```
轴A: 舵机转向 (SG90, G14, LEDC Timer0 Channel0)
轴B: 双马达独立驱动 (L=G1/G11/G12, R=G2/G9/G10, LEDC Timer2 Channel1/3)
输入: PS2 无线手柄 (DAT=G4, CMD=G5, CLK=G7, CS=G6)
```

---

## 2. 两个模式

| | 模式1 后驱 (Rear) | 模式2 前驱 (Front) |
|---|---|---|
| 车头 | 轴A (舵机) | 轴B (马达) |
| 转向方式 | 舵机 | 马达差速 |
| 驱动方式 | 轴B 双马达同速 | 轴B 双马达差速 |
| 轴A 角色 | 转向 | 从动拖行（舵机回中） |
| 物理操作 | 正常方向 | **调转车身 180°** |

---

## 3. 输入映射

### 模式 Rear（后驱）

| 输入 | 映射 | 死区 |
|------|------|------|
| 右摇杆 X | 舵机转向角 | ±3 |
| 左摇杆 Y | 双马达同速（比例） | ±3 |

```
steering = rx_to_deg(rx, STEER_DEG=60)
speed    = ly_to_speed(ly)       // 0→+2048, 128→0, 255→-2048
left     = speed
right    = speed
```

### 模式 Front（前驱差速）

| 输入 | 映射 | 死区 |
|------|------|------|
| 右摇杆 X | 左右差速 | ±3 |
| 左摇杆 Y | 基础速度（比例） | ±3 |

```
base = ly_to_speed(ly)
diff = centered_rx * MAX_DIFF / 127   // MAX_DIFF = 2048
left  = clamp(base + diff, -4095, 4095)
right = clamp(base - diff, -4095, 4095)
舵机 → center
```

---

## 4. 全部决策

| # | 议题 | 决定 |
|---|------|------|
| 1 | 架构模式 | `enum DriveMode { Rear, Front }` + match |
| 2 | 模式切换按钮 | L3 + R3 同时按下 → 停车 → 翻转 |
| 3 | 差速幅度 MAX_DIFF | 2048 (= MOTOR_DUTY) |
| 4 | 油门 | 左摇杆 Y 比例控制 |
| 5 | R1/L1 | 废弃 |
| 6 | 马达反转保护 | slew-rate (512/tick) + coast-before-reverse |
| 7 | 舵机 Front 模式 | 每 tick 更新 center |
| 8 | joystick.rs | 删除 |
| 9 | 代码结构 | 新 `control.rs` 承载策略，main 为协调层 |
| 10 | 摇杆死区 | RX ±3, LY ±3 |
| 11 | MotorSlew 位置 | `control.rs` |
| 12 | Embassy 多任务 | 搁置 |

---

## 5. 代码结构

```
src/
├── bin/main.rs       ← 硬件初始化 + 循环骨架 (~120行)
├── control.rs        ← 所有控制策略（纯函数 + MotorSlew + DriveMode）
├── steering.rs       ← 不改
├── tb6612.rs         ← 不改（纯硬件驱动）
├── ps2.rs            ← 不改
└── lib.rs            ← pub mod control; (移除 joystick)
```

### 5.1 control.rs — 纯函数

```rust
pub fn ly_to_speed(ly: u8) -> i32 { ... }      // 左摇杆 → 速度，内置死区
pub fn rx_to_deg(rx: u8, max_deg: i32) -> i32 { ... }  // 右摇杆 → 角度，内置死区
pub fn motor_rear(ly: u8) -> (i32, i32) { ... }  // Rear: 同速
pub fn motor_front(ly: u8, rx: u8) -> (i32, i32) { ... }  // Front: 差速
```

### 5.2 control.rs — MotorSlew

```rust
pub struct MotorSlew {
    current_l: i32, current_r: i32,  // slew-rate 追踪
    last_l: i32,    last_r: i32,     // 反转保护追踪
    max_step: i32,
}

impl MotorSlew {
    pub fn new(max_step: i32) -> Self;
    pub fn update(&mut self, target_l: i32, target_r: i32) -> (i32, i32);
    pub fn reset(&mut self);
}
```

双层保护：

1. **slew-rate**: 每 tick 速度变化 ≤ 512 counts → 全油门约 130ms
2. **coast-before-reverse**: 符号翻转时插入一帧 coast (IN1=IN2=0)

### 5.3 control.rs — 枚举 + 配置

```rust
#[derive(Clone, Copy, PartialEq, defmt::Format)]
pub enum DriveMode { Rear, Front }
impl DriveMode { pub fn flip(self) -> Self { ... } }

pub struct ControlConfig {
    pub steer_max_deg: i32,   // 60
    pub motor_max_duty: i32,  // 2048
    pub motor_slew_step: i32, // 512
    pub rx_deadzone: i32,     // 3
    pub ly_deadzone: i32,     // 3
}
```

---

## 6. main.rs 骨架

```rust
use jocar_steer::control::{self, ControlConfig, DriveMode, MotorSlew};

#[esp_rtos::main]
async fn main(_spawner: Spawner) -> ! {
    // --- 硬件初始化（~60行，不变） ---
    // RTT, esp_hal, LEDC timer+channel 配置, PS2, Steering, Tb6612

    let config = ControlConfig {
        steer_max_deg: 60, motor_max_duty: 2048,
        motor_slew_step: 512, rx_deadzone: 3, ly_deadzone: 3,
    };
    let mut mode = DriveMode::Rear;
    let mut motor_slew = MotorSlew::new(config.motor_slew_step);
    let mut steering = Steering::new(ch, 3, config.steer_max_deg);
    let mut motors = Tb6612::new(/* ... */);

    loop {
        let state = ps2.read();

        // PS2 analog 检测/恢复（不变）
        if !state.is_analog() { /* hold & wait for MODE */ continue; }

        // 模式切换: L3+R3
        if state.pressed(Button::L3) && state.pressed(Button::R3) {
            motors.set_left(0); motors.set_right(0);
            steering.center();
            mode = mode.flip();
            motor_slew.reset();
            Timer::after(Duration::from_millis(33)).await;
            continue;
        }

        // 转向
        steering.set_target(control::rx_to_deg(state.rx(), config.steer_max_deg));
        steering.update(8);

        // 马达
        let (l_target, r_target) = match mode {
            DriveMode::Rear  => control::motor_rear(state.ly()),
            DriveMode::Front => {
                steering.set_target(0); // 舵机回中
                control::motor_front(state.ly(), state.rx())
            }
        };
        let (l, r) = motor_slew.update(l_target, r_target);
        motors.set_left(l);
        motors.set_right(r);

        Timer::after(Duration::from_millis(33)).await;
    }
}
```

---

## 7. 改动清单

| 文件 | 操作 |
|------|------|
| `src/control.rs` | **新建** |
| `src/lib.rs` | + `pub mod control;` − `pub mod joystick;` |
| `src/joystick.rs` | **删除** |
| `src/bin/main.rs` | **重写** |
| `src/steering.rs` | 不改 |
| `src/tb6612.rs` | 不改 |
| `src/ps2.rs` | 不改 |

## 8. 实现顺序

1. 删除 `joystick.rs` + 更新 `lib.rs`
2. 新建 `control.rs`
3. 重写 `main.rs`
4. `cargo build` 验证
