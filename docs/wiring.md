# 接线（当前整车 + 历史）

## 当前：src/bin/main.rs（单电机，2026-08）

- SG90 舵机：**G14**（LEDC Timer0/Ch0, 50 Hz）
- TB6612FNG 通道 A（单马达）：
  - PWMA / **G13**（LEDC Timer2/Ch1, 10 kHz）
  - AIN1 / **G11**
  - AIN2 / **G12**
  - STBY / **G10**
  - (VCC / 3V3, GND / GND, VM / 电池 7.4V, AO1 / 马达+, AO2 / 马达-)
- ZD USB 手柄接收器（native OTG 口）：
  - D- / **G19**, D+ / **G20**
  - VBUS / 5V buck 输出（背板 **USB-OTG 焊盘已短接**，与板 5VIN 同网，2S 下为规范 5V）
- 电源：
  - 2S 7.4V 锂电 ── 直通 ── TB6612/AT8236 VM
  - 2S 7.4V ── **MP1584EN 固定 5V**（输入 4.5-28V，标称 3A）── 板 5VIN（AMS1117→3.3V）+ 舵机 + 接收器 VBUS

## 历史 / 其他固件

### archive/ps2-dual-motor-car.rs（PS2 双模车，旧底盘，已归档）

- PS2 接收器：CLK / G7, CS / G6, CMD / G5, DAT / G4（VCC / 3V3）
- 舵机：G14（LEDC T0/Ch0）
- 双马达：右 PWMA / G1, 左 PWMB / G2；AIN1 / G9, AIN2 / G10, BIN1 / G11, BIN2 / G12, STBY / G13

### MAX98357A（待接线，未用）

- SD / G17, GAIN / 3V3, DIN / G38, BCLK / G39, LRC / G40（VCC / 3V3）
