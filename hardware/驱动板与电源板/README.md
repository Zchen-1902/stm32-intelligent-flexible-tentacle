# 三电机驱动板与电源板

## 文件

- `FOC无刷驱动板.eprj2`：嘉立创 EDA 专业版工程源文件。

## 作用

该工程对应单个触手的三电机执行单元，围绕 STM32G474、三路无刷电机栅极驱动、电流采样、编码器接口、CAN/FDCAN 通信及系统供电构建底层实时控制硬件。

## 与软件的对应关系

- 固件目录：`firmware/g4-motor-controller`
- CubeMX 工程：`firmware/g4-motor-controller/FOC_G474VET6_CONFIG.ioc`
- Keil 工程：`firmware/g4-motor-controller/MDK-ARM/FOC_G474VET6_CONFIG.uvprojx`

生产和调试前应重点检查 MOSFET 与驱动器额定值、栅极回路、电流采样增益、采样电阻功率、母线电压、散热、保险与反接保护。首次上电应使用限流电源，并先验证驱动器寄存器、PWM 互补输出和过流保护，再连接实际负载。
