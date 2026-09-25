# STM32G474 三电机 FOC 控制工程

这是从 `D:\STM32Cube_project\4.8` 整理出的独立 G4 控制板工程。目录内包含完整源码、STM32 HAL/CMSIS、CMSIS-DSP 数学库、CubeMX 配置和 Keil MDK 工程。

## 打开和编译

1. 使用 Keil MDK 打开 `MDK-ARM/FOC_G474VET6_CONFIG.uvprojx`。
2. 选择工程目标 `FOC_G474VET6_CONFIG` 后执行 Build。
3. 如需修改引脚、时钟或外设，使用 STM32CubeMX 打开 `FOC_G474VET6_CONFIG.ioc`。

## 构建验证

本独立副本已使用 Keil MDK 5.40、Arm Compiler 5.06 update 6 完成一次全量构建：

```text
0 Error(s), 11 Warning(s)
Code=49034, RO-data=2830, RW-data=228, ZI-data=6660
```

可直接烧录的 `hex`、调试用 `axf` 和链接映射 `map` 已复制到 `Release/`。构建警告来自当前业务源码中的未使用调试变量、未引用备用函数和浮点类型提升，不是独立工程缺少文件造成的。

## 当前关键配置

- MCU：STM32G474VET6
- 当前板配置：`G4_BOARD_PROFILE_ID = 4U`
- 当前 CAN 板号：`CAN_APP_BOARD_ID = 3U`
- PWM/电流环目标频率：10 kHz

烧录其他 G4 板之前，应在 `Core/motor/Config.h` 中选择对应的 `G4_BOARD_PROFILE_ID`。各板的 CAN ID、电机反馈方向和电角度补偿集中定义在 `Core/motor/board_profiles.h`。

## 主要模块

- `Core/motor`：三环控制、FOC、SVPWM和板级参数
- `Core/adc_app`：三电机相电流采样
- `Core/spi_encoder`：三路磁编码器读取
- `Core/drv8323S_drive`：三路栅极驱动器配置
- `Core/can_app`：与 H7 主控的 CAN FD 通信
- `Core/Vofa_debug`：VOFA JustFloat 调试输出
- `Core/timer_app`：低频定时任务

`Core`、`Drivers`、`Middlewares` 和 `MDK-ARM` 的相对目录关系不能改变，否则 Keil 工程中的相对路径将失效。
