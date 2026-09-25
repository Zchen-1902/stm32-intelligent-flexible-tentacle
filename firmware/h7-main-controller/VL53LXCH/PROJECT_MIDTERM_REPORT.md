# STM32G474 三电机控制项目中期记录

## 1. 项目目标

本项目基于 STM32G474VET6，目标是驱动 3 个 BM352006 无刷电机，用于带减速器和线轮的拉索式机构。当前阶段主要任务是搭建三电机控制的软件基础框架，在功率板和部分传感器未完全焊接前，优先完成可独立验证的软件模块、调试链路和控制算法基础。

当前项目暂不以直接带载运行电机为目标，而是先保证：

- 软件结构清晰，后续三电机扩展方便。
- VOFA 调试链路稳定，能够观察关键变量。
- 定时器、ADC、SPI、DMA 等底层链路逐步独立验证。
- FOC 数学计算和 PWM CCR 写入逻辑先在无功率输出条件下验证。
- 编码器、DRV、电流采样等硬件相关模块先搭建接口，等待硬件具备条件后实测。

## 2. 当前硬件资源规划

主要硬件资源分配如下：

- `TIM1`：电机 1 三相互补 PWM。
- `TIM8`：电机 2 三相互补 PWM。
- `TIM20`：电机 3 三相互补 PWM。
- `ADC1`：电机 1 三相电流采样。
- `ADC3`：电机 2 三相电流采样。
- `ADC4`：电机 3 三相电流采样。
- `ADC2`：温度采样。
- `SPI1`：三路 DRV8323S/DRV8328 驱动器配置。
- `SPI3`：三路 MT6816 磁编码器读取。
- `USART1`：VOFA JustFloat 调试输出。
- `FDCAN1`：后续与上位主控通信。
- `TIM4`：5ms 系统软件调度基础节拍。
- `TIM7`：当前设置为 10ms 编码器采样/速度计算节拍。
- `TIM6`：预留给 ADC2 温度采样节拍。

## 3. 当前软件目录结构

当前新增或重点维护的软件模块如下：

- `Core/Vofa_debug`：VOFA JustFloat 调试输出模块，使用 USART1 DMA 发送浮点数据。
- `Core/timer_app`：定时器应用层，将 TIM4/TIM6/TIM7 中断转换为主循环可处理的软件任务标志。
- `Core/adc_app`：ADC DMA 数据组织与回调入口，负责三路电机电流 ADC 的原始采样缓存。
- `Core/spi_encoder`：MT6816 SPI 编码器驱动，已实现 SPI3 DMA 串行读取三路编码器的框架。
- `Core/drv8323S_drive`：DRV 驱动配置接口框架，目前以头文件和基础结构为主。
- `Core/motor/foc_drive.c/.h`：FOC 数学与 SVPWM 输出通路，包含 Clarke/Park/InvPark/SVPWM 和 CCR 写入。
- `Core/motor/Config.h`：当前阶段主要配置参数，包括电机、编码器、PID 默认参数、限幅和调试相关配置。
- `Core/motor/Motor_type.h`：电机相关结构体类型定义。

## 4. 当前已完成内容

### 4.1 VOFA 调试链路

USART1 + DMA 的 VOFA JustFloat 调试链路已经跑通，可以稳定显示波形。当前默认调试主题仍以 `VOFA_TOPIC_FOC_TEST` 为主，用于显示 FOC/SVPWM 测试数据和 CCR 输出。

当前 FOC 测试主题主要通道为：

- `I0`：测试电角度。
- `I1`：实际电角度预留。
- `I2`：SVPWM 扇区。
- `I3`：d 轴归一化电压。
- `I4`：q 轴归一化电压。
- `I5`：TIM1 CCR1。
- `I6`：TIM1 CCR2。
- `I7`：TIM1 CCR3。

### 4.2 定时任务框架

`timer_app` 已经建立：

- `TIM4` 当前作为 5ms 基础调度节拍。
- `TIM7` 当前作为编码器采样/速度计算节拍。
- 中断中只置任务标志，具体任务在主循环中执行，避免中断负担过重。

当前 `TIM7` 的 CubeMX 公式为：

```c
htim7.Init.Period = 100000 / SPEED_CALCU_FREQ - 1;
```

当前 `Config.h` 中：

```c
#define SPEED_CALCU_FREQ 10
```

因此当前 TIM7 实际周期为 10ms。

### 4.3 ADC DMA 框架

`adc_app` 已建立，用于组织 `ADC1`、`ADC3`、`ADC4` 的原始采样数据和 DMA 完成标志。当前阶段 ADC 真实电流输入尚未完整接入，因此只能验证 ADC/DMA 链路，暂不能验证实际电流值、零漂、电流方向和比例系数。

当前适合验证：

- ADC DMA 是否启动。
- DMA 完成回调是否进入。
- raw buffer 是否更新。

当前不适合验证：

- 三相电流换算是否准确。
- 零漂校准是否真实有效。
- DRV 电流采样输出比例是否正确。

### 4.4 FOC/SVPWM 输出通路

`foc_drive` 已实现第一版 FOC 输出通路：

- `FOC_Clarke()`：三相电流到 alpha/beta。
- `FOC_Park()`：alpha/beta 到 d/q。
- `FOC_InvPark()`：d/q 电压到 alpha/beta。
- `FOC_SVPWM()`：alpha/beta 归一化电压到 SVPWM duty。
- `FOC_WritePwmDuty()`：将 duty 写入 TIM CCR。
- `FOC_RunVoltageNorm()`：执行一次归一化电压 FOC 计算。

当前已通过 VOFA 验证：

- 测试电角度可以周期扫描。
- SVPWM sector 可以 1~6 循环。
- CCR1/CCR2/CCR3 会随 duty 变化。
- 当前只写 CCR，不启动 PWM 输出，不控制 DRV，不接功率板。

需要注意：当前 `VOFA_TOPIC_FOC_TEST` 显示的 `sector` 是 SVPWM 输入电压矢量 `Ualpha/Ubeta` 所在扇区，不一定等于转子电角度所在扇区。当测试参数为 `vd=0, vq>0` 时，电压矢量相对电角度有 90 度偏移。

### 4.5 SPI 编码器 DMA 框架

`spi_encoder` 已实现三路 MT6816 编码器读取框架：

- SPI3 配置为 8bit、Mode 3、MSB first。
- 使用 DMA Normal 模式。
- 三路编码器通过 CSN1/CSN2/CSN3 串行读取。
- 每次 TIM7 任务触发后，在主循环启动一次三路编码器 DMA 读取。
- SPI DMA 完成后通过 `HAL_SPI_TxRxCpltCallback()` 转发到 `SPI_Encoder_TxRxCpltCallback()`。
- 三路 raw 读取完成后统一更新角度、速度、多圈计数。

当前编码器数据流为：

```text
TIM7 10ms节拍
-> main启动 SPI_Encoder_Start_Update_All_DMA()
-> SPI3 DMA依次读取 encoder1/encoder2/encoder3
-> 解析MT6816 raw
-> raw转机械角
-> 处理零点和方向
-> 更新单圈角、多圈角、turn_count、speed
```

MT6816 手册已核对：

- `0x03 = Angle[13:6]`
- `0x04[7:2] = Angle[5:0]`
- `0x04[1] = No_Mag_Warning`
- `0x04[0] = PC 偶校验位`

当前尚未接真实编码器实测，因此 raw 连续性、方向、零点、多圈和速度仍需后续硬件验证。

### 4.6 DRV 驱动框架

`drv8323S_drive` 已建立基础文件，当前主要完成驱动配置接口的规划和头文件框架。SPI1 使用 8bit 两字节方式更适合当前 HAL 配置，第一版不使用 SPI1 DMA。

后续需要补充：

- DRV 寄存器读写。
- DRV 基础初始化。
- CAL 引脚校准函数。
- 关键寄存器配置。
- nFAULT/状态寄存器读取。

## 5. 当前主循环与中断关系

当前核心回调链路如下：

```text
TIM4中断
-> HAL_TIM_PeriodElapsedCallback()
-> Timer_App_PeriodElapsedCallback()
-> TIMER_APP_TASK_5MS
-> main中调用 VOFA_Debug_Task()

TIM7中断
-> HAL_TIM_PeriodElapsedCallback()
-> Timer_App_PeriodElapsedCallback()
-> TIMER_APP_TASK_SPEED_CALC
-> main中调用 SPI_Encoder_Start_Update_All_DMA()

SPI3 DMA完成
-> HAL_SPI_TxRxCpltCallback()
-> SPI_Encoder_TxRxCpltCallback()

SPI3错误
-> HAL_SPI_ErrorCallback()
-> SPI_Encoder_ErrorCallback()

ADC DMA完成
-> HAL_ADC_ConvCpltCallback()
-> ADC_App_ConvCpltCallback()

USART1 DMA发送完成
-> HAL_UART_TxCpltCallback()
-> VOFA_Debug_TxCpltCallback()
```

当前设计原则是：中断中只做短任务或回调转发，复杂处理尽量放在主循环或模块内部的低频任务中。

## 6. 当前未验证项与风险点

当前项目仍有以下未验证项：

- 编码器硬件尚未焊接，MT6816 SPI 数据读取未实物验证。
- 编码器速度计算当前使用固定软件时间戳，后续可升级为基于真实 DMA 完成时间戳的每路更新时间。
- ADC 电流采样链路未接真实电流采样前端，无法验证实际电流换算。
- DRV8323S 配置和 CAL 校准函数尚未完整实现。
- TIM1/TIM8/TIM20 触发 ADC 的实际采样时序需要后续结合硬件和 ADC 数据确认。
- FOC 当前只验证数学输出和 CCR 写入，没有启动真实 PWM 输出。
- PWM 互补输出、死区、DRV 使能、功率级保护尚未联调。
- FDCAN 上位机协议尚未实现。
- PID 三环控制算法尚未开始正式编码。

## 7. Git 记录

当前已建立 Git 仓库，并已整理 `.gitignore`，Keil 编译产物不再纳入版本控制。

近期关键提交：

- `1202a47`：initial project snapshot
- `5727d52`：2026-04-09 initial project snapshot
- `1ea4329`：2026-04-11 vofa working and key exti falling edge update
- `6f2a58d`：add timer app 5ms scheduler
- `38f559f`：add adc app and foc vofa validation
- `879bfc7`：Add SPI encoder DMA and driver framework
- `ce05f83`：Ignore Keil build outputs
- `aaa3b3e`：Set TIM7 encoder task period to 10ms

当前重要快照：

```text
snapshot-before-tim7-encoder-dma
```

## 8. 后续开发计划

近期建议开发顺序如下：

1. 完成 `VOFA_TOPIC_ENCODER` 的实际数据填充，并支持选择查看某一路编码器。
2. 在不接硬件的情况下继续验证 `VOFA_TOPIC_FOC_TEST` 和 CCR 输出是否稳定。
3. 开始编写控制算法层模块，包括 PID、限幅、积分限幅、抗积分饱和、死区、斜坡和滤波。
4. 建立单电机 `motor_core` 软件链路，先使用假数据验证数据流。
5. 建立最小单电机状态机，包括 OFF、INIT、CALIB、READY、RUN、FAULT。
6. 硬件焊接完成后，依次实测编码器、ADC电流采样、DRV配置和PWM输出。
7. 先完成单电机开环，再进入电流环闭环。
8. 单电机链路稳定后，再扩展到三电机统一调度。

## 9. 当前建议

在硬件尚未完整具备条件前，当前最适合继续推进的软件内容是控制算法层：

- `control_pid`
- `control_ramp`
- `control_deadband`
- `control_limit`
- `control_filter`

这些模块不依赖真实硬件，可以通过假输入和 VOFA 波形验证。完成后再接入单电机 `motor_core`，为后续真实闭环控制打基础。
