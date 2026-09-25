# 基于示教学习的多触手智能柔性末端执行器

本项目面向柔性抓取与仿生作业场景，采用 STM32H743 作为主控制器、STM32G474 作为三电机 FOC 从控制器，集成 VL53L8CH ToF 感知、端侧 CNN/TCN 推理、CAN/FDCAN 通信、上位机控制与示教数据采集功能。

## 目录结构

```text
firmware/
  g4-motor-controller/   STM32G474 三电机 FOC 控制工程
  h7-main-controller/    STM32H743 感知、推理和多触手控制工程
pc/
  tentacle-controller/   多触手运动控制上位机
  act-data-collector/    ACT 示教数据采集上位机
training/
  gesture-cnn/           手势识别 CNN 训练代码
  act-tcn/               ACT/TCN 策略训练与 ONNX 导出代码
models/                  最终部署模型
docs/                    项目文档
```

## 主要功能

- STM32H743 完成 ToF 数据采集、模型推理、控制调度和状态管理。
- STM32G474 完成三台无刷电机的编码器读取、FOC 控制和安全保护。
- 主从控制器通过 CAN/FDCAN 交换目标位置与电机状态。
- 上位机支持单触手、双触手和四触手控制，以及运行状态可视化。
- ACT 数据采集工具同步保存 ToF 帧和三电机位置，用于示教策略训练。
- 端侧部署手势 CNN 与 Delta-TCN 两套模型。

## 工程入口

### G4 电机控制器

- CubeMX：`firmware/g4-motor-controller/FOC_G474VET6_CONFIG.ioc`
- Keil：`firmware/g4-motor-controller/MDK-ARM/FOC_G474VET6_CONFIG.uvprojx`

### H7 主控制器

- CubeMX：`firmware/h7-main-controller/H743VIT6.ioc`
- Keil：`firmware/h7-main-controller/MDK-ARM/H743VIT6.uvprojx`

### 上位机与训练代码

各 Python 子工程中的 `README.md`、`requirements.txt` 和入口脚本给出了对应的运行方式。训练数据、编译中间文件和本地缓存未纳入仓库。

## 模型

- `models/gesture_openness_cnn.onnx`：手势开合识别模型。
- `models/delta_tcn_scene6_epoch010.onnx`：单触手示教策略模型。

## 说明

仓库保留可复现工程所需的源码、配置和最终模型，未收录 Keil 编译产物、第三方桌面工具、训练数据集及本地临时文件。
