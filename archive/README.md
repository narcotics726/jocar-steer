# archive/ — 已废弃代码

不参与 cargo 构建（不在 `src/bin/` 顶层）。完整历史版本由 git 保留，此处仅作可读存档。

| 文件 | 原位置 | 说明 |
| --- | --- | --- |
| `ps2-dual-motor-car.rs` | `src/bin/main.rs` | PS2 双马达双模车固件（Servo/Diff 双模）。2026-08 被单电机 + USB 手柄固件取代（新主固件在 `src/bin/main.rs`）。对应设计文档见 `docs/archive/design-dual-mode.md`、`docs/archive/hardware-layout.md`。 |
| `lighting/ws2812_stat_indicator.rs` | `src/lighting/` | WS2812 状态机（`StatusInput` + 闪灯/淡入优先级）。整个形态是双模 + PS2 时代的产物：输入含 `DriveMode`、模拟 PS2 断连。随 `DriveMode` 删除一并归档；其中的 `Rgb` 类型被两个 bring-up 工具用着，已就地内联到那两个 bin。灯效真需要时再按当期状态重写（WS2812 时序经验在 git 历史里）。 |
| `ws2812-test.rs` | `src/bin/ws2812-test.rs` | 上面那个状态机的演示 bin（Servo/Diff/PS2 断连五阶段）。 |
