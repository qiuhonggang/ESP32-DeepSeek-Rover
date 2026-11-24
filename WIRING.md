# 四驱小车接线

适用 ESP32-S3 N16R8、两颗 INMP441、3070 喇叭配 MAX98357A、四个 TT 电机、两块 DRV8833、HC-SR04 和 18650 供电。表中的数字是 GPIO 编号。对照自己开发板的丝印，尤其确认 GPIO17、GPIO18 已引出。

## 引脚表

| 模块端子 | 接到 ESP32 或电源 | 备注 |
| --- | --- | --- |
| 两颗 INMP441 VDD，GND | 3.3 V，GND | 每颗电源就近 0.1 μF 去耦，部分成品板已带 |
| 两颗 INMP441 SCK，WS | GPIO4，GPIO5 | 与功放共用 BCLK 和 WS 两条时钟 |
| 两颗 INMP441 SD | GPIO6 | 两颗 SD 共接，数据线上加一只到 GND 的 100 kΩ 下拉 |
| 左麦、右麦 L/R | 左麦 GND，右麦 3.3 V | 对应代码左右声道 |
| MAX98357A BCLK，LRC，DIN | GPIO4，GPIO5，GPIO7 | 采用标准 I²S 的 MAX98357A |
| MAX98357A VIN，GND | 稳定 5 V，GND | 模块的 SD/EN 按板上默认使能接法 |
| MAX98357A SPK+，SPK− | 3070 喇叭的两根线 | 桥接功放两端都接喇叭，SPK− 勿接系统 GND |
| HC-SR04 VCC，GND | 稳定 5 V，GND | 固定在车头 |
| HC-SR04 TRIG | GPIO8 | 若模块无法可靠识别 3.3 V TRIG，补电平转换 |
| HC-SR04 ECHO | **分压后接 GPIO9** | Echo → 10 kΩ → GPIO9，GPIO9 → 15 kΩ → GND，5 V 分到约 3.0 V |
| DRV8833 A 的 AIN1，AIN2 | GPIO10，GPIO11 | AOUT1/AOUT2 接左前轮 |
| DRV8833 A 的 BIN1，BIN2 | GPIO12，GPIO13 | BOUT1/BOUT2 接右前轮 |
| DRV8833 B 的 AIN1，AIN2 | GPIO14，GPIO15 | AOUT1/AOUT2 接左后轮 |
| DRV8833 B 的 BIN1，BIN2 | GPIO17，GPIO18 | BOUT1/BOUT2 接右后轮 |
| 两块 DRV8833 nSLEEP | GPIO16 | 共享使能，建议外接 10 kΩ 下拉。模块若把 nSLEEP 硬绑电源，按实际板图处理 |
| 两块 DRV8833 VM，GND | 电机动力支路，GND | 每个 H 桥驱动一个电机 |

如果模块用 `SLEEP`、`SLP` 或 `EN` 等不同标注，先确认它对应 DRV8833 的低电平休眠端，且不会向 GPIO16 回灌电机电压。3070 只描述了扬声器外形规格之一，驱动前查看背面的额定阻抗和功率。功放首次测试用固件默认低音量。 [INMP441 手册](https://invensense.tdk.com/wp-content/uploads/2015/02/INMP441.pdf) · [MAX98357A 手册](https://www.analog.com/media/en/technical-documentation/data-sheets/max98357a-max98357b.pdf) · [HC-SR04 手册](https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf) · [DRV8833 手册](https://www.ti.com/product/DRV8833)

## 供电分支

如果 TT 电机的额定电压允许从 1S 锂电池直接取电，使用带过充、过放和过流保护的匹配电池组。1S 标称约 3.7 V、充满约 4.2 V。电池组可采用已有的合适保护电池，容量与并联节数按实际负载决定。需要另备匹配的 1S 充电电路及稳压 5 V 模块，按其说明连接，关机充电。

```text
已保护 1S 18650 电池组
  │
  ├── 匹配的 1S 充电电路（按电池组标注的充电端连接）
  │
  └── 保险与总开关
       ├── 电机动力支路  → DRV8833 A、DRV8833 B 的 VM → 四个 TT 电机
       └── 5 V 稳压支路 → ESP32 开发板 5V/VIN、MAX98357A、HC-SR04
                              └─ ESP32 3.3 V → 两颗 INMP441
所有控制模块、传感器和电源支路共用参考地。
```

- 查实际 TT 电机数据或用限流电源短时测量起步和堵转电流。核对每路 H 桥、两路同时带载时的板级散热、保护板、开关、线材和电池供电能力。DRV8833 的峰值电流与可持续电流不同，板上芯片封装也会影响可持续电流。 [TI DRV8833 数据](https://www.ti.com/product/DRV8833)
- 5 V 模块先空载调好，再验证 Wi-Fi 和功放同时运行时电压稳定。稳压模块选型要看 1S 低压时的持续输出和负载跳变。不要把 5 V 送进 ESP32 的 3V3 端。
- 电机驱动和 5 V 支路从开关后的分叉点取电，电机回流远离麦克风线。驱动板和稳压输出的去耦按模块资料布置，电机电源线尽量短。首次烧录时断开外部 5 V 和电池动力，避免与 USB 供电并接。 [ESP32-S3 DevKitC 供电说明](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32s3/esp32-s3-devkitc-1/user_guide_v1.1.html)
- 电池组放在底板上方靠中间，用绝缘固定带和防滑垫固定，按实际尺寸预留走线。截图随底盘附送的是四节 AA 电池盒，18650 使用独立电池组或合适的固定件。

## 机械安装与上电顺序

1. 两个 INMP441 放在车头两侧，拾音孔中心距先用 **100 mm**。拾音孔朝向尽量一致并保持通畅。实测间距填进网页，喇叭放到另一端，减少直接耦合。
2. 超声波装在车头中央、探头朝前。相同地面条件下先放一块较宽的平板检验测距，再尝试椅腿、门边等障碍。近距离或侧面的障碍可能在探测范围之外。
3. 先只接逻辑供电，核对每条电源轨和 Echo 分压，再确认双麦、UART、喇叭和超声波。四轮架空接入动力，短按每个轮子的点动键。通过后再做低速地面标定和避障。
