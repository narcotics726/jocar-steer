# 双模控制 — 实现任务清单

> 状态: pending | in_progress | done

---

## T1: 删除 joystick.rs + 更新 lib.rs

**状态**: ✅ done
**依赖**: 无
**验证**: `cargo build` 通过

---

## T1.5: 改造 ps2.rs — Ps2Event 模式

**状态**: ✅ done
**依赖**: 无
**验证**: `cargo build --bin ps2-test` 通过

---

## T2: 新建 control.rs

**状态**: ✅ done
**依赖**: 无
**验证**: `cargo build` 通过

---

## T3: 重写 main.rs

**状态**: ✅ done
**依赖**: T1, T1.5, T2
**验证**: `cargo build` 通过

---

## T4: 全量构建验证

**状态**: ✅ done
**依赖**: T3

```bash
cargo build          # ✅ passed
cargo build --bin ps2-test  # ✅ passed
```

## 最终文件状态

| 文件 | 状态 |
|------|------|
| `src/joystick.rs` | **已删除** |
| `src/lib.rs` | `pub mod control;` + 移除 `joystick` |
| `src/ps2.rs` | 新增 Ps2Event + was_analog + read_raw |
| `src/bin/ps2_test.rs` | 适配 Ps2Event API |
| `src/control.rs` | **新建**: ControlConfig, DriveMode, 4 纯函数, MotorSlew |
| `src/bin/main.rs` | **重写**: 使用 Ps2Event 事件 + control 层映射 + 双模式 |
| `src/steering.rs` | 不改 |
| `src/tb6612.rs` | 不改 |
