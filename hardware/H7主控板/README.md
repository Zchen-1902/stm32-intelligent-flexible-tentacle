# H7 主控板

## 文件

- `h743vit6.eprj2`：嘉立创 EDA 专业版工程源文件。

## 作用

该板以 STM32H743VIT6 为核心，承担系统上层控制任务，包括 VL53L8CH 多区域感知、手势 CNN 与 ACT Delta-TCN 推理、多触手目标调度、CAN/FDCAN 通信、串口调试、外部存储和运行状态管理。

## 与软件的对应关系

- 固件目录：`firmware/h7-main-controller`
- CubeMX 工程：`firmware/h7-main-controller/H743VIT6.ioc`
- Keil 工程：`firmware/h7-main-controller/MDK-ARM/H743VIT6.uvprojx`

制造和上电前应依据原理图复核电源域、时钟、下载接口、通信接口和外设引脚，并确认固件配置与当前 PCB 版本一致。
