# jocar-steer 架构重构计划 v3

**版本**: v3（2026-10）
**状态**: 讨论定案，待实施
**修订链**: v1 计划 → 独立复审（7/10）→ v2 逐项定案 → v3 并入 1/10 车移植需求与跨线结论合流

---

## 0. 背景与立场

旧 `main.rs`（PS2 双模车）确立了分层抽象：输入层 → 策略层（纯函数 + `MotorSlew`）→ 执行器层 → main 装配 + 轮询。

立场：**本计划不是"恢复原设计"**——原设计本身有隐藏缺陷（超级循环、per-tick 单位错误）。原则：**策略层收编、计算/施加分离、时间基准语义、可测性（按需）、安全兜底、每项抽象按真实迭代史付租金**。

- 双模废弃 = 软件随硬件的自然迁移；`DriveMode`/`motor_servo`/`motor_diff` 随旧固件归档。
- **目标从一台车变成两台车**：旧车（TB6612 + SG90 + USB 手柄）与 1/10 车（有刷电调 + MG996R + 同一手柄）。这使"输入边界"与"执行器抽象"从预防性设计变成当期需求。
- **no_std 立场**：选的是 esp-hal 裸金属路线（非 esp-idf/std 路线）；async 的承重用途在 **USB host 层**（embassy-usb-host 是 async 原生，去不掉）；**控制层用循环是形状正确**（周期顺序控制不需要任务交错）。

---

## 1. 已执行

**main.rs 归档**（构建已验证 `cargo build --bin jocar-steer` 通过；改动尚**未提交**，含两个未跟踪文件）：

| 变更 | 详情 |
| --- | --- |
| 旧主固件 | `src/bin/main.rs` → `archive/ps2-dual-motor-car.rs`（不再参与构建） |
| 新主固件 | `src/bin/usb-gamepad-car.rs` → `src/bin/main.rs`（bin 名回落 `jocar-steer`） |
| 过期文档 | `docs/design-dual-mode.md`、`docs/hardware-layout.md` → `docs/archive/` |
| 清单 | `Cargo.toml` 删 `usb-gamepad-car` 的 `[[bin]]`；引用同步（`wiring.md`、`verification-wiring.md`） |
| 存档说明 | 新增 `archive/README.md` |

待办：新 `main.rs` 模块注释里 "mirrors the PS2 car in `main.rs`" 已过时，随 Phase 0 一并改。

---

## 2. 已定案决策（共享核心）

### 2.1 P1-1 时间基准语义 —— **活 bug，热修先行**

- **bug（确认存在，仍在车上）**：`STEER_SLEW_STEP=8°/tick` 在报告驱动下失效——@125Hz 报告 = 1000°/s（设计 242°/s 的 4×），@1000Hz = 8000°/s（33×，形同虚设）；`KICK_TICKS=6` 从 ~200ms 缩到 6–48ms。电机侧 slew 同理。
- slew 改 `°/s`、`speed/s`；kick 改 `ms`；用 embassy `Instant`/`Duration` 结算。
- **dt 由调用方传入、不封顶**（久断后恢复应立刻响应）；µs 粒度用 **u64/i64**。
- **API 纯函数化**：dt 作参数（同输入必同输出），为 P2-4 实验预留可测性。
- **常量迁移**：`STEER_SLEW_STEP` → `cfg.steer_slew_rate_deg_s`；`motor_slew_step` → `cfg.motor_slew_rate_speed_s`；`KICK_TICKS` → `cfg.kick_duration_ms`。
- **命名精化**（随之而来）：control 层的 `*_duty` → `*_speed`（**抽象速度单位 ±4095**）。TB6612 里 1:1 映射为占空比，电调里按比例映射为脉宽——这正是计算/施加分离的落地。
- **kick 结构**：改 `Instant` 比较（触发记 `kick_until`），**不是**每报告递减 `ticks_left`（否则只改单位无效）。

### 2.2 P1-2 Chassis + `MotorDriver` trait

- 新模块 **`src/chassis.rs`**（不能进 `control.rs`——后者须保持零 esp-hal 依赖）。
- 结构：`steering: Steering<S>`、`motors: M`（**驱动类型，非通道类型**）、`slew`、`kick`、`target_speed`、`cfg`、`last_report: Option<Instant>`。
- **API（轴值边界）**：
  ```rust
  pub fn on_report(&mut self, steer_axis: u8, throttle_axis: u8, now: Instant);
  pub fn tick(&mut self, dt: Duration);
  pub fn halt(&mut self);
  ```
  dt/now 一律由调用方传入（确定性、可测）；断开与输入陈旧共用 `halt()`；瞬时滑行。
- **`MotorDriver` trait（租金到期：两个真实实现 TB6612 / 电调）**：
  ```rust
  pub trait MotorDriver {
      fn set_speed(&mut self, speed: i32); // 有符号抽象速度
      fn stop(&mut self);                  // TB6612: coast(duty 0)；Esc: 中位脉冲
      fn enable(&mut self);                // TB6612: STBY 高；Esc: 开始输出中位
  }
  ```
  **语义差异必须由各实现兜住**——这是 trait 存在的核心理由：电调的"停车"是**持续发中位脉冲**，给 duty 0 等于信号丢失。
- **输入契约**：轴值 0..255、128 中心。未来换输入（ELRS/CRSF 等）= 新输入模块 + bin 调用点，`control.rs`/Chassis/执行器零改动。
- failsafe 阈值 `cfg.failsafe_timeout_ms`。

### 2.3 P1-3 计算/施加分离

- `control.rs` 纯函数：`steer_deg(rx, cfg)`、`throttle_duty→throttle_speed(ly, steer, cfg) -> i32` **一步返回最终值**（含混控削减 + clamp）。
- **混控用目标转向角**（现状行为保留；实际角更"物理准确"但引入时变）。
- `StartKick` 搬进 `control.rs`；`control.rs` 保持零 esp-hal 依赖。
- **删 `DriveMode`/`motor_servo`/`motor_diff`**——涟漪已查明：唯一库外消费者是 `lighting::ws2812_stat_indicator`（其 `StatusInput` 含 `mode: DriveMode` 字段）与 `ws2812-test`（模拟 Servo/Diff/PS2 断连五阶段）。处置见 2.12。

### 2.4 安全项（全部定案）

**A. panic handler**：`loop {}` → `defmt::error!` + `esp_hal::system::software_reset()`。

**B. 分阶段 WDT**：
- 用 **TIMG1**（空闲；TIMG0.timer0 被 esp-rtos 占用）；`timg::Wdt`，Stage0 默认动作 = `ResetSystem`。
- **控制阶段 WDT = 1s，非控制阶段（连接/枚举/hub 等待）关闭**。依据：电机只在控制阶段可能被驱动（进入非控制阶段前必经 `halt()` 或处于开机初始态）。
- **喂狗在控制循环内**（独立任务喂狗会让 WDT 测不到主循环挂死）。
- **硬前提：读超时 2s → 500ms**（最长合法不喂狗间隔必须 < WDT 超时）。
- hub `wait_for_event` 属非控制阶段（可合法无限阻塞），同待遇。
- 代价：非控制阶段挂死无自动恢复（等同现状，纯改进无回归）。

**C. `wait_for_connection` 加 5s 超时重试**。

**failsafe 窗口 5s → 2s**（`cfg.failsafe_timeout_ms`）——对 1/10 车从"优化"升级为**刚需**（见 3.5）。

**硬件注记**：复位/开机瞬间 TB6612 STBY 浮空（无 PWM → 不驱动，风险低）；可选加下拉电阻。

### 2.5 kick

- 旧车：`cfg.kick_duration_ms`，**先设 0 禁用**，实车 A/B（起步响应 + 反复起步温度）再定去留。
- **1/10 车：直接关**（见 3.3 边界条件②，kick 残留会破坏电调中位判定）。

### 2.6 P2-4 主机测试 = B

推迟到 Phase 1 完成后做 **30 分钟 path-include 实验**（`#[path]` 引入 `control.rs`，零改动主 crate）再定；**不做拆 crate**。前置：`control.rs` 零 esp-hal 依赖（已定）。esp-hal 在 host 上硬失败的既有事实不构成阻塞（纯逻辑层不碰它）。

### 2.7 P3-5 InputState = 砍掉

控制层纯函数已输入无关；复审建议的 `{lx,ly,rx,ry,buttons}` 是 USB 形状，未来 CRSF（通道制）不适用。**替代 = 2.2 的轴值边界**。未来真出现第二输入源时再加，纯加法。

### 2.8 P4-7 任务分解 = 砍掉

| 原承诺价值 | 已被谁解决 |
| --- | --- |
| 节拍漂移 | P1-1 时间基准 |
| kick 时长漂移 | P1-1 Instant 比较 |
| 报告停控制就停 | 安全项（读超时 500ms + failsafe 2s） |
| 代码组织 | P5-8 |

成本（`embassy-sync` 版本矩阵、大改动）无对价收益。Chassis 的 `on_report`/`tick` 分离保留结构性后路。

### 2.9 P3-6 显式状态机 = **溶解**

原动机（`mode_switch_held` 边沿检测）随双模删除一并消失；连接状态由 `usb_session` 的循环结构表达；`StartKick.prev_was_zero` 由 Instant 计时取代；`last_report` 归 `Chassis::failsafe`。无需单列实施项。

### 2.10 P5-8 硬件映射 + USB 会话（两半）

- **半边 A（降级版）**：不建 `hardware.rs`；抽 `reserve_system_pins` 辅助（GPIO 保留段样板）+ 各 bin 自带 pin-map 注释，与 `docs/wiring.md` 互链。两 bin 后去重价值上升。
- **半边 B（硬需求）**：抽 **`src/usb_session.rs`**，两 bin 共用：`wait_for_connection`（含 5s 超时）、枚举、hub fallback、读循环、500ms 读超时、failsafe 归还 Chassis、WDT 喂狗与分阶段开关、重枚举判定。两 bin 各自只剩"初始化 + 构造 Chassis + `run()`"。

### 2.11 错误处理策略

**保持 `.unwrap()`**（初始化处）+ 明文策略：**启动期配置失败 = panic = 复位（日志可见）；运行期不 panic**。约束：将来任何依赖**运行时数据**的初始化不得 unwrap。理由：现有 unwrap 覆盖的皆是编译期常量错误（不可达），完整错误传播对 no_std 初始化不值。

### 2.12 测试 bin 与 `lighting` 处置

已查明依赖面（grep 全部 bin）：`ps2-test` 只依赖 `ps2.rs`（本次不动）→ 不会断；`pcf8575-test` 完全不 import 本库 → 不会断；`servo-test`/`battery-mon-test`/`usb-gamepad-*` 不受影响；**唯一会断的是 `ws2812-test`**（用 `control::DriveMode`）。

- **不急于归档 bin**（两个"死" bin 不断、无害，归档属装修）。
- **真正的决定**：`lighting` 模块 + `ws2812-test` ——该状态指示器**整场是双模 + PS2 时代的产物**（`StatusInput` 含 `DriveMode`、模拟 Servo/Diff/PS2 断连），而**当前固件根本没 import lighting**（死代码）。→ **随时代一起归档**；1/10 车真需要指示灯时再写一个极小的新的（WS2812 时序经验在 git 历史里）。
- `servo-test` **保留并扩展为脉宽扫描/标定工具**（两车都要用；1/10 车的电调端点与倒车最小停留靠它标定）。
- `battery-mon-test` 保留（见 3.6 低电保护）。

### 2.13 AGENTS.md 瘦身（原则：不放易腐知识）

| 内容 | 处置 |
| --- | --- |
| File Layout（文件/模块清单） | **删**（典型易腐） |
| "esp-rtos (based on embassy-executor)" | **改**——是事实错误，非细节：前置 RTOS + embassy 集成 |
| Current car hardware 整节 | **瘦身**：留耐久判据（12V 电机在 2S 属欠压→满占空比安全；发热来自堵转电流而非电压），删逐脚枚举，指向 `docs/wiring.md` |
| Build & Flash / Toolchain / Constraints | **留**（稳定：构建命令、GPIO 保留、clippy 禁令、`cargo test` 是 build 别名） |

**不加任何清单**（bin 列表、模块列表、per-car 台账）。

---

## 3. 1/10 车移植

### 3.1 目标车与对照

| | 旧车（jocar） | 1/10 车 |
| --- | --- | --- |
| 底盘 | 手搓单后驱 | HSP 94123 一族仿件；四驱；轴距 ~255mm；胎径 64mm |
| 马达 | 12V 额定 N30 4000RPM（2S 下欠压） | 540 有刷 |
| 马达控制 | TB6612 通道 A（方向 + 10kHz PWM） | **有刷电调 BDESC-S10E-RTR（HSP03018）**，50Hz 脉冲 |
| 转向 | SG90 @ G14 | **MG996R @ G14** |
| 输入 | USB 手柄（G19/G20） | 同（不换） |
| 电池 | 2S 7.4V | 2S |

### 3.2 `Esc` 驱动器（新，`src/esc.rs`）

- **接口**：50Hz，脉宽 1.0 / 1.5 / 2.0 ms 标称（**端点可配**：`neutral_us` / `span_us`，因电调而异）。
- **解锁时序**：上电/复位后**立即输出中位并保持**（解锁窗口），期间不得发油门。**与安全项 A/B 的交互**：panic/WDT 复位后 LEDC 停 → 电调按"信号丢失"自行收油（对电调车是安全方向），但复位后须**重走解锁序列**。
- **中位死区**：|speed| 低于阈值时**精确发中位**，否则车蠕动。
- **两段式倒车**：`last` 为前进/刹车且新指令为倒车时，**先插 `cfg.reverse_dwell_ms`（默认 ~300ms）的确切中位**，再发倒车脉冲。最小停留时长**待实测**（判据：电调 LED 灭 = latch 已复位，逐步缩短找最小值）。
- **`stop()` = 持续中位脉冲**（不是 duty 0）。
- **LED 语义**（免费的调试读出器）：有输出亮、中位灭。
- 分辨率：与舵机共用一个 50Hz 定时器；若电调手感偏粗，可提高该定时器的 duty 分辨率（13–14 bit）。

### 3.3 两段式倒车的三条边界（必须写进固件设计）

1. **油门死区要够宽**——盖住手柄回中残差（经验 ±2–5/255），保证"松手 = 恰好 1.5ms"。太窄 ⇒ 电调永远等不到中位 ⇒ 倒车彻底不出，现象与"电调没倒车"一模一样，极难查。
2. **油门通道不得有任何残余**——无 idle creep、无慢衰减残值、**无 kick 残留**；松手必须干净回中位。
3. **倒车不得由程序触发**（当前边界）——若将来要脚本化倒车/自动脱困，必须自己实现状态机（本方案已实现 300ms 前置中位，届时可直接复用）。

**待实测的预测**：两段式只作用于"从前进过来"的换向 ⇒ **从静止中位直接掰倒车应一次就进**。若从静止也进不去，说明另有原因，别急着改固件。

### 3.4 参数差异（1/10 车）

| 参数 | 值 | 理由 |
| --- | --- | --- |
| 混控 mix | **关（0）** | 混控是"无差速 + 低减速比 → 拖磨堵转"的补偿；1/10 车有正常转向几何与传动 |
| kick | **0** | 见 3.3 边界②；且电调自带软启动 |
| 转向限位 | 放宽（~45–60°，**待实车标定**） | 同 mix 理由；标定前先保守 |
| 油门上限 | 首次测试保守值（**待定**） | 540 + 2S 与 N30 完全不同的功率量级 |
| 中位偏置 | 重新标定 | 两车几何差约一倍 |

### 3.5 硬件与供电（定案）

```
2S ─┬─ 降压 A(3A级) ── 舵机          （就地 +470–1000µF + 0.1µF）
    ├─ 降压 B(1A级) ── ESP32 5VIN
    └─ 直供 ── 电调 ── 马达           （电调 BEC 红线悬空、绝缘；三处共地）
```

- **电调 BEC 一概不用**：廉价电调的 BEC 常兼喂自身逻辑，MG996R 堵转 1.5–2.5A（BEC 标 2A）拉垮时**电调自己可能跟着复位** → 不是转向变弱而是整车瞬间失控；且大脑独立供电后，电调复位时 ESP32 仍能立刻收油。
- 实测数据支撑：MG996R 堵转 1.5–2.5A；BEC 标 5V/2A；MP1584 一类"标 3A、实际连续按 ~2A"。
- 舵机端电容就地（堵转是瞬态）；**转向行程不得顶死机械限位**（顶死 = 持续堵转，电容扛不住）。
- **调试期坑**：TTL 口 USB 会给板子 5V（clone 板 USB 5V 直连 5V 轨），与降压同接 5VIN 是两路电源打架——插 USB 调试时先拔降压，或用只走数据的线。
- **未采纳的单路变体**（记录备查）：`2S → 单颗 5V(3A级) → 舵机+ESP32`。风险链：舵机堵转使 5V 节点下跌 >0.6V → AMS1117（需 ≥4.4V）崩 → ESP32 复位。若要试，判据：ESP32 跑日志时硬堵转舵机数秒，看有无断档/重启；但即便通过，仍建议分路（外部堵转现场难复现）。

### 3.6 电调油门引脚与 LEDC 资源

- **电调信号 = G1**（旧双马达 PWMA/PWMB 已空出；与 TB6612 的 G13 不冲突，满足"两套输出并跑"；一行可换）。
- 舵机与电调均为 50Hz → **共用同一个 LEDC 定时器**（Timer0 两个通道），Timer2 空出。
- **低电保护（潜在需求）**：该电调**无确定 LiPo 截止**——`battery.rs` 已有电池监测模块，可考虑在 1/10 车上做低电压收油/告警（未定案）。

---

## 4. 实施顺序（路线 A）

**Phase 0 — 提交与热修**
1. 提交归档改动 + 清理 `main.rs` 过时注释。
2. **P1-1 时间基准热修**（~30 行）+ `kick_duration_ms=0` + 安全项 A（panic→复位）。

**Phase 1 — 共享核心重构**
3. P1-3：`control.rs` 收编（`throttle_speed` 含 mix、`StartKick` 入库）+ 删 `DriveMode` → 处置 `lighting`/`ws2812-test`（2.12）。
4. P1-2：`src/chassis.rs` + `MotorDriver` trait + `Tb6612Single` 实现。
5. 安全项 B/C + 读超时 500ms + failsafe 2s。
6. P5-8：`src/usb_session.rs` 抽取 + `reserve_system_pins`。

**Phase 2 — 1/10 车 bin**
7. `src/esc.rs`（含解锁、死区、300ms 前置中位）。
8. `src/bin/rc10.rs` 骨架 + 引脚（G14 舵机 / G1 油门）+ 参数（3.4）。
9. `servo-test` 扩展为脉宽扫描/标定工具。
10. **上车标定**：电调端点与中位、倒车最小停留、MG996R 端点、油门死区宽度、转向限位。

**Phase 3 — 收尾**
11. P2-4 的 30 分钟 path-include 实验。
12. AGENTS.md 瘦身（2.13）。

---

## 5. 风险与约束

- **WDT 约束**：WDT 超时必须大于控制循环内最长合法 await（现为读超时 500ms → WDT 1s 为 2× 余量）。**未来加长 await 时必须同步改 WDT 超时**（写进代码注释）。
- **失联窗口的量级**：1/10 车全油门 2 秒 ≈ 16 米；现状 5s 窗口在该车上不可接受——安全项不是优化。
- **倒车状态机的隐性失效**：死区过窄/存在残留 ⇒ 倒车永久不出且现象与硬件故障同形（3.3 三条边界）。
- **电调解锁与复位**：任何复位后须重走解锁序列；调试期不要让电调在无信号与有信号之间反复切换。
- **标定依赖**：端点/中位/最小停留/限位都必须实车标定，代码里先给保守默认值并集中放 `cfg`。
- **精度**：时间结算用 u64/i64（µs 粒度下 i32 边缘）。
- **无测试状态**：kick 手感/温度、电调行为只有实车能回答；host 测试（若做）只覆盖语义不变量。
- **归档不参与构建**：`archive/` 不在 `src/bin/` 顶层，cargo 不扫描。

---

## 6. v1 独立复审要点与采纳情况

复审总评 7/10：

| 复审要点 | 采纳 |
| --- | --- |
| P2-4 机制错误（esp-hal host 硬失败，cfg_attr 不足够，前置四条） | ✅（2.6） |
| 无安全项（WDT/panic）——最大遗漏 | ✅ 定案（2.4） |
| P1-1 是活 bug 非潜在 bug | ✅（2.1） |
| kick 只改单位无效（需 Instant 比较） | ✅（2.1） |
| esp-rtos 是 RTOS 非 embassy-executor 分发；embassy 0.10 无每任务栈 | ✅（2.13） |
| P4-7 前置应为 P1-1，建议降级可选 | ✅ 升级为砍掉（2.8） |
| P5-9 自相矛盾 + 5s 超时双职责拆分 | ✅（2.2/2.4/2.10） |
| P3-5 建议 InputState 数据 struct | ⚠️ 部分采纳——改为轴值边界（2.7） |
| 遗漏：main.rs 去留、测试 bin 处置、AGENTS.md、错误处理 | ✅（1、2.11–2.13） |
| 精度：µs 粒度 i32 边缘 | ✅（2.1） |

---

## 7. 未决与待实测

- **bin 命名**：1/10 车 bin 暂定 `src/bin/rc10.rs`。
- **待实测值**：电调端点/中位、倒车最小中位停留、MG996R 端点、油门死区宽度、1/10 转向限位与首次油门上限。
- **未定案（记录备查）**：profile 跳线选择器（已选两 bin 方案，暂不做）；电调低电保护（`battery.rs` 可用）；单路降压变体（未采纳，判据在 3.5）。
