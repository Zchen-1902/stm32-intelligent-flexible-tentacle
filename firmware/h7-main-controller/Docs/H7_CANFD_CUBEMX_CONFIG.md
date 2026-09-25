# H7 CAN FD CubeMX 配置说明

## 1. 目标

H7 主控板通过 FDCAN1 与 G4 电机从板通信。

当前通信目标：

```text
H7 -> G4:
  发送 0x120 CAN FD 位置命令帧
  一帧携带 motor0 / motor1 / motor2 三路目标位置

G4 -> H7:
  接收 0x180 CAN FD 状态帧
```

要求 H7 和 G4 的 CAN FD 位速率一致：

```text
Nominal bitrate: 1 Mbit/s
Data bitrate:    5 Mbit/s
Frame format:    CAN FD with BRS
```

## 2. 当前 H7 需要修改的原因

当前 H7 工程生成结果仍是 Classic CAN：

```c
hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
hfdcan1.Init.NominalPrescaler = 10;
hfdcan1.Init.NominalTimeSeg1 = 13;
hfdcan1.Init.NominalTimeSeg2 = 2;
hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
```

这会导致：

```text
1. 无法发送/接收 32 字节 CAN FD 帧
2. 无法使用 BRS 数据段加速
3. 与 G4 当前 CAN FD 配置不匹配
4. PD0/PD1 GPIO 速度过低，不适合 5Mbit/s 数据段
```

## 3. H7 FDCAN kernel clock

当前 H7 CubeMX 配置中 FDCAN 使用 PLL2 作为内核时钟。

当前生成配置：

```c
PeriphClkInitStruct.PLL2.PLL2M = 5;
PeriphClkInitStruct.PLL2.PLL2N = 64;
PeriphClkInitStruct.PLL2.PLL2P = 2;
PeriphClkInitStruct.PLL2.PLL2Q = 4;
PeriphClkInitStruct.PLL2.PLL2R = 4;
PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL2;
```

CubeMX 当前计算出的 FDCAN 位时序显示：

```text
Nominal Time Quantum = 125 ns
```

说明当前 FDCAN kernel clock 用于位时序计算约为：

```text
1 / 125 ns = 8 MHz time quantum base after prescaler effect in current config
```

结合当前配置：

```text
Nominal Prescaler = 10
Nominal bit time = 2000 ns
Nominal bitrate = 500 kbit/s
```

可反推出当前 FDCAN kernel clock 为：

```text
80 MHz
```

所以 H7 不能照抄 G4 的 85MHz 位时序参数，但最终的仲裁段和数据段 bitrate 必须与 G4 一致。

## 4. CubeMX 中 FDCAN1 参数

在 `Connectivity -> FDCAN1` 中修改。

### 4.1 基本模式

```text
Frame Format:          FD mode with BitRate Switching
Mode:                  Normal mode
Auto Retransmission:   Enable
Transmit Pause:        Disable
Protocol Exception:    Disable
```

含义：

```text
FD mode with BitRate Switching:
  使用 CAN FD，并在数据段切换到高速 bitrate。

Auto Retransmission:
  发送失败、仲裁丢失或错误后自动重发。

Transmit Pause:
  第一版关闭，避免额外发送间隔影响轨迹下发。

Protocol Exception:
  第一版关闭，保持协议行为简单。
```

### 4.2 仲裁段 Nominal Bit Timing

目标：

```text
Nominal bitrate = 1 Mbit/s
```

H7 当前 FDCAN kernel clock 约为 80 MHz，推荐：

```text
Nominal Prescaler:        5
Nominal Sync Jump Width:  3
Nominal Time Seg1:        12
Nominal Time Seg2:        3
```

计算：

```text
80 MHz / 5 / (1 + 12 + 3) = 1 MHz
```

采样点：

```text
(1 + TimeSeg1) / (1 + TimeSeg1 + TimeSeg2)
= (1 + 12) / 16
= 81.25%
```

### 4.3 数据段 Data Bit Timing

目标：

```text
Data bitrate = 5 Mbit/s
```

推荐：

```text
Data Prescaler:        1
Data Sync Jump Width:  3
Data Time Seg1:        12
Data Time Seg2:        3
```

计算：

```text
80 MHz / 1 / (1 + 12 + 3) = 5 MHz
```

## 5. Message RAM 配置

CAN FD 32 字节帧必须把 Rx/Tx element size 改大。

推荐配置：

```text
Std Filters Nbr:          1 或 8
Ext Filters Nbr:          0

Rx FIFO0 Elements Nbr:    8
Rx FIFO0 Element Size:    32 bytes

Rx FIFO1 Elements Nbr:    0
Rx FIFO1 Element Size:    32 bytes 或默认

Tx FIFO Queue Elements:   8
Tx Element Size:          32 bytes
Tx FIFO Queue Mode:       FIFO mode
```

生成后应看到：

```c
hfdcan1.Init.RxFifo0ElmtsNbr = 8;
hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_32;
hfdcan1.Init.TxFifoQueueElmtsNbr = 8;
hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_32;
```

如果仍是：

```c
FDCAN_DATA_BYTES_8
```

则 H7 无法正确收发 32 字节 CAN FD 帧。

## 6. GPIO 配置

当前 H7 使用：

```text
PD0 -> FDCAN1_RX
PD1 -> FDCAN1_TX
```

CubeMX 中把 PD0/PD1 的 GPIO speed 改为：

```text
Very High
```

生成后应看到：

```c
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
```

不要保持：

```c
GPIO_SPEED_FREQ_LOW
```

5Mbit/s 数据段下 GPIO 低速边沿可能导致通信不稳定。

## 7. NVIC 配置

当前 H7：

```text
FDCAN1_IT0_IRQn priority = 5
```

第一版可以保持。

H7 是主控，不运行 G4 那种高频电流环，因此 FDCAN 中断优先级可以比 G4 稍高。但仍建议中断中只收帧缓存，不做复杂轨迹规划。

## 8. 生成后 fdcan.c 检查清单

CubeMX 生成代码后，检查 `H743VIT6/Core/Src/fdcan.c`。

应满足：

```c
hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
hfdcan1.Init.AutoRetransmission = ENABLE;

hfdcan1.Init.NominalPrescaler = 5;
hfdcan1.Init.NominalSyncJumpWidth = 3;
hfdcan1.Init.NominalTimeSeg1 = 12;
hfdcan1.Init.NominalTimeSeg2 = 3;

hfdcan1.Init.DataPrescaler = 1;
hfdcan1.Init.DataSyncJumpWidth = 3;
hfdcan1.Init.DataTimeSeg1 = 12;
hfdcan1.Init.DataTimeSeg2 = 3;

hfdcan1.Init.RxFifo0ElmtsNbr = 8;
hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_32;
hfdcan1.Init.TxFifoQueueElmtsNbr = 8;
hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_32;
```

GPIO 应满足：

```c
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
```

## 9. 与 G4 配置对齐

G4 当前配置：

```text
FDCAN kernel clock after divider: 85 MHz
Nominal: 1 Mbit/s
Data:    5 Mbit/s
FD+BRS:  enabled
```

H7 当前推荐配置：

```text
FDCAN kernel clock: 80 MHz
Nominal: 1 Mbit/s
Data:    5 Mbit/s
FD+BRS:  enabled
```

注意：

```text
H7 和 G4 的具体 Prescaler/TimeSeg 可以不同
但最终 Nominal bitrate 和 Data bitrate 必须一致
```

## 10. H7 发送 0x120 命令帧

H7 发给 G4：

```text
ID:      0x120
Format:  CAN FD + BRS
DLC:     32 bytes
```

Payload：

```text
byte0      protocol_version = 1
byte1      command = 0x01 position / 0x02 stop
byte2      flags
byte3      motor_mask = 0x07
byte4      seq
byte5..7   reserved
byte8..11   motor0_target_q int32 little-endian
byte12..15  motor1_target_q int32 little-endian
byte16..19  motor2_target_q int32 little-endian
byte20..31  reserved
```

位置缩放：

```text
target_q = motor_target_rad * 100000
```

常用 flags：

```text
ENABLE + APPLY = 0x09
ENABLE + APPLY + RELATIVE = 0x0D
STOP = 0x02
```

## 11. H7 接收 0x180 状态帧

G4 返回给 H7：

```text
ID:      0x180
Format:  CAN FD + BRS
DLC:     32 bytes
```

Payload：

```text
byte0    protocol_version
byte1    board_id
byte2    online
byte3    control_word
byte4..5 rx_count low16
byte6..7 tx_count low16
byte8..11 error_count uint32
byte12   pwm_started_mask
byte13   last_relative_seq
byte14   last_relative_seq_valid
byte15   position_pending
byte16   last_position_seq
byte17   last_position_flags
byte18   last_position_motor_mask
byte19   last_position_command
byte20..31 reserved
```

## 12. 调试顺序

1. 先只观察 G4 状态帧：

```text
是否能看到 0x180
是否为 CAN FD + BRS
DLC 是否为 32
```

2. H7 发送无动作位置帧：

```text
ID 0x120
flags = 0
motor_mask = 0x07
```

预期：

```text
G4 0x180 rx_count 增加
G4 error_count 不增加或只因 flags 不满足而按预期增加
电机不动
```

3. H7 发送 stop 帧：

```text
command = 0x02
flags = 0x02
motor_mask = 0x07
```

4. H7 发送小幅绝对位置命令：

```text
command = 0x01
flags = 0x09
motor_mask = 0x07
```

先使用接近当前位置的小目标，避免触手大幅抽动。

## 13. 注意事项

1. H7 和 G4 必须使用相同的 CAN FD 物理层设置：

```text
Nominal 1M
Data 5M
BRS ON
终端电阻正确
CAN FD 收发器支持 5M 数据段
```

2. 电脑 CAN 分析器也要配置为：

```text
CAN FD
Nominal 1M
Data 5M
BRS ON
```

3. 如果总线能看到错误帧，优先检查：

```text
波特率
GPIO speed
收发器是否支持 CAN FD
终端电阻
H7/G4 是否共地
Rx/Tx element size 是否为 32 bytes
```
