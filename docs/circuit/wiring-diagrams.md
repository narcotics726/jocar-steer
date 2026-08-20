# 接线图 — 分域版（2026-08）

> 单张图塞全部信息必然乱。按域拆成四张小图，每张只有 5-7 条边。
> 焊线时以 `verification-wiring.md` 表格为准（对照实物），本图用于连通性检查。

## ① 电源

```mermaid
flowchart LR
  BAT["2S 电池 7.4V"] -->|"BAT+ → IN+"| BUCK["MP1584EN 固定5V"]
  BAT -->|"VM 7.4V"| DRV["TB6612"]
  BUCK -->|"5V"| ESP["ESP32 5VIN"]
  BUCK -->|"5V"| SERVO["舵机 +"]
```

## ② 地线（星形）

```mermaid
flowchart BT
  GND["共地点 ★"]
  BAT["电池 -"] --> SW["开关 3A"] --> GND
  BUCK["降压 IN-/OUT-"] --> GND
  DRV["TB6612 GND"] --> GND
  ESP["ESP32 GND"] --> GND
  SERVO["舵机 -"] --> GND
```

## ③ 信号

```mermaid
flowchart LR
  ESP["ESP32-S3"] -->|G11| AIN1["AIN1"]
  ESP -->|G12| AIN2["AIN2"]
  ESP -->|G13| PWMA["PWMA"]
  ESP -->|G10| STBY["STBY"]
  ESP -->|3V3| VCC["VCC"]
  ESP -->|G14| SIG["舵机信号"]
```

## ④ 电容（每颗 = 轨 ↔ 地）

```mermaid
flowchart LR
  VM["VM 轨"] --- C1["1000µF/16V"] --- G["地"]
  5V["5V 轨"] --- C2["1000µF/16V"] --- G
  5V --- C3["470µF"] --- G
  BAT["电池+"] --- C4["1000µF/16V"] --- G
  M["马达端子"] --- C5["100nF"]
```

## 备注

- GPIO 为实接值：AIN1=G11 AIN2=G12 PWMA=G13 STBY=G10 舵机=G14
- 开关 3A/250VAC：2S 下踩线可用，3S 必须换 ≥5A/12VDC
- 全部电容负脚统一回共地点（单点地）
