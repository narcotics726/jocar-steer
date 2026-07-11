# 转向抽象重构指导计划（Steering）

目标：把「转向」这一行为从 `main.rs` 的 loop 里抽出来，封装成一个与输入源无关、
可复用的 `Steering` 类型。执行层只认「角度」，将来输入源从 MCP23017 按键换成
PS2 摇杆时，`Steering` 一行都不用改。

本次**只做直接 `set_angle` 版本**，斜率限速（slew-rate）只留接口位、不实现。

---

## 1. 分层原则（不要破坏的契约）

```
输入层（会变：按键 → PS2）  ──产出「期望转向角(带符号, 正=左)」──▶  Steering（稳定）
```

- `Steering` **不知道**角度是按键给的还是摇杆给的。
- `Steering` 内部封装：中位偏移、单侧限位、角度→counts 换算、±90° 安全钳位、写 PWM。
- 输入层的唯一职责：算出一个「期望角度」，交给 `Steering::set_angle`。

---

## 2. 文件与模块结构

- 新建 `src/steering.rs`。
- 在 `src/lib.rs` 中加：`pub mod steering;`（`lib.rs` 目前只有 `#![no_std]`）。
- 从 `main.rs` **移走**以下内容到 `steering.rs`，作为模块私有细节：
  - 常量：`PERIOD_US`、`DUTY_MAX`、`CENTER_US`、`US_PER_90DEG`
  - 函数：`angle_to_counts`

---

## 3. 类型设计

```rust
use esp_hal::ledc::channel::ChannelHW;

pub struct Steering<Ch> {
    channel: Ch,
    center_deg: i32,   // 静态中位偏移（原 CENTER_TRIM_DEG）
    max_deg: i32,      // 单侧最大打角（原 STEER_DEG）
    trim_deg: i32,     // 运行时微调
    target_deg: i32,   // 最近一次 set_angle 的输入（供 adjust_trim 重新应用）
}
```

- 泛型约束 `Ch: ChannelHW`：不绑死 LEDC 具体类型，便于替换/脱机测试。
- 注意 `ChannelHW::set_duty_hw(&self, duty: u32)` 是 `&self`，所以即便方法签名是
  `&mut self` 也能正常调用。
- `Channel<'a, LowSpeed>` 自带生命周期，由泛型参数 `Ch` 一并携带，无需在 `Steering`
  上显式写生命周期。

---

## 4. 方法清单与行为（只描述行为，本次不写实现体）

| 方法 | 签名 | 行为 |
|------|------|------|
| 构造 | `new(channel: Ch, center_deg: i32, max_deg: i32) -> Self` | 初始化字段，`trim_deg=0`、`target_deg=0`，构造后立即 `apply()` 让舵机回到中位 |
| 设角 | `set_angle(&mut self, deg: i32)` | 记 `target_deg = deg.clamp(-max_deg, max_deg)`，然后 `apply()` |
| 回中 | `center(&mut self)` | 等价 `set_angle(0)` |
| 微调 | `adjust_trim(&mut self, delta: i32)` | `trim_deg` 累加并 clamp（建议 ±30），然后 `apply()` 让偏移立即生效 |
| 查询 | `trim(&self) -> i32` | 返回当前 `trim_deg`（供日志） |
| 私有 | `apply(&mut self)` | 计算并写 PWM，见下 |

`apply()` 的计算链（核心公式）：

```
effective_deg = center_deg + trim_deg + target_deg
counts        = angle_to_counts(effective_deg)   // 内部 clamp 到 ±90°(1000~2000µs)
channel.set_duty_hw(counts)
```

`angle_to_counts`（从 main 搬来，保持不变）：

```
deg      = deg.clamp(-90, 90)
pulse_us = CENTER_US + deg * US_PER_90DEG / 90     // 1500 + deg*500/90
counts   = DUTY_MAX * pulse_us / PERIOD_US         // 4096 * us / 20000
```

参考值（用于自检）：`0°→307`、`+90°→410`、`-90°→205`、`+60°→375`、`+3°(静态中位)→310`。

---

## 5. 斜率限速的预留位（本次**不**实现，仅留注释说明）

在 `steering.rs` 顶部或 `set_angle` 附近写一段 doc-comment，说明将来若需要柔和转向：

- 新增 `set_target(deg)` 只存目标、不立即写；
- 新增 `update(max_step)`（在固定 tick 里调用）把一个内部 `current_deg` 朝 `target_deg`
  每次挪动至多 `max_step`；
- `max_step` 足够大时行为等价于当前的立即 `set_angle`。

⚠️ 结论仍是：**转向舵一般不需要限速**（要跟手），且 PS2 模拟摇杆天然连续。此层留给
油门/电机或按键版手感优化，届时再实现。

---

## 6. `main.rs` 迁移步骤

1. 删除已移走的常量和 `angle_to_counts`。
2. 保留 `const STEER_DEG` 和 `const CENTER_TRIM_DEG`（构造时传入，按键映射也复用 `STEER_DEG`）。
3. 配好 `ch`（LEDC channel）后构造：
   ```rust
   let mut steering = Steering::new(ch, CENTER_TRIM_DEG, STEER_DEG);
   ```
4. loop 退化为薄薄的「按键 → 期望角度」映射（临时输入层）：
   ```rust
   if pin0 { steering.adjust_trim(-1); }   // X
   if pin1 { steering.adjust_trim(1); }    // A
   let cmd = if pin3 { STEER_DEG }         // Y 左
             else if pin2 { -STEER_DEG }   // B 右
             else { 0 };                   // 松开回中
   steering.set_angle(cmd);
   ```
5. 日志改成从 `steering.trim()` 取值；`last_deg` 去重逻辑可保留在 main，或直接每 tick 写。

---

## 7. 验证

- `cargo build` 通过。
- 烧录后行为应与当前完全一致：Y 左 / B 右 / X·A 微调 / 松开回中，且中位带 3° 偏移。
- 可选：用 RTT 日志核对几个角度的 counts 是否等于第 4 节参考值。
- 由于 `[lib] test = false`，如需单元测试 `angle_to_counts`，走 `embedded-test`
  （dev-deps 里已有，参照 `tests/hello_test.rs` 的 `harness = false` 写法）。

---

## 8. 为 PS2 输入层预留的接口形态（下一阶段，不在本次范围）

本次抽象成立的验证：PS2 摇杆升级时，**只替换第 6 节第 4 步的输入映射**，`Steering` 不动。

```rust
// PS2 模拟摇杆 (0..=255, 128 居中) → 比例转向，天然平滑
let cmd = (stick_x as i32 - 128) * STEER_DEG / 128;
steering.set_angle(cmd);
```
