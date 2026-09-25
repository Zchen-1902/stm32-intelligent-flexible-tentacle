# H7-G4 CAN FD 串口触发闭环测试说明

## 1. 测试目标

本测试用于验证整条通信链路是否打通：

```text
电脑串口助手 / VOFA
  -> USART1
  -> H7
  -> CAN FD 0x120
  -> G4
  -> CAN FD 0x180
  -> H7
  -> USART1
  -> 电脑串口助手 / VOFA
```

该测试的目标不是验证触手运动算法，也不是验证最终模型推理，而是先确认：

1. H7 能通过 USART1 收到上位机命令。
2. H7 能发送 CAN FD + BRS 的 0x120 控制帧。
3. G4 能收到并解析 H7 的 0x120 帧。
4. G4 能发送 CAN FD + BRS 的 0x180 状态帧。
5. H7 能收到并解析 G4 的 0x180 帧。
6. H7 能把结果通过 USART1 打印给上位机。

## 2. CAN FD 物理层和 CubeMX 配置要求

H7 和 G4 必须保持一致：

```text
Frame Format: CAN FD with BRS
Nominal bitrate: 1 Mbit/s
Data bitrate:    5 Mbit/s
Data field size: 32 bytes
```

硬件要求：

```text
CAN FD 收发器支持 5M 数据段
总线两端 120R 终端电阻正确
H7 和 G4 共地
CANH/CANL 连接正确
线尽量短，先做桌面短线测试
```

## 3. H7 发给 G4 的命令帧

H7 发送：

```text
ID:      0x120
Format:  Standard ID
Frame:   CAN FD + BRS
DLC:     32 bytes
```

payload：

```text
byte0       protocol_version = 1
byte1       command
byte2       flags
byte3       motor_mask
byte4       seq
byte5..7    reserved
byte8..11   motor0_target_q int32 little-endian
byte12..15  motor1_target_q int32 little-endian
byte16..19  motor2_target_q int32 little-endian
byte20..31  reserved
```

命令定义：

```text
command = 0x01  position target
command = 0x02  stop
```

flags：

```text
bit0 ENABLE = 0x01
bit1 STOP   = 0x02
bit3 APPLY  = 0x08
```

第一版位置命令使用：

```text
command = 0x01
flags = 0x09
motor_mask = 0x07
```

停止命令使用：

```text
command = 0x02
flags = 0x02
motor_mask = 0x07
```

位置缩放：

```text
target_q = motor_target_rad * 100000
motor_target_rad = target_q / 100000.0f
```

示例：

```text
CANPOS 1000 0 0
```

表示：

```text
motor0 = 0.01 rad
motor1 = 0
motor2 = 0
```

## 4. G4 需要做什么

G4 侧至少需要实现以下逻辑。

### 4.1 接收 0x120

G4 应配置 FDCAN 接收标准 ID：

```text
0x120
```

收到帧后检查：

```text
ID == 0x120
DLC == 32 bytes
FD == enable
BRS == enable
byte0 protocol_version == 1
```

若检查通过，解析：

```text
command
flags
motor_mask
seq
motor0_target_q
motor1_target_q
motor2_target_q
```

### 4.2 更新接收统计

G4 收到有效 0x120 后应更新：

```text
rx_count++
last_position_seq = seq
last_position_flags = flags
last_position_motor_mask = motor_mask
last_position_command = command
```

如果帧格式错误，可以更新：

```text
error_count++
```

### 4.3 执行策略

闭环测试第一阶段建议不要直接大幅运动。

推荐策略：

1. `command = 0x02 STOP` 时进入停止或保持状态。
2. `command = 0x01 POSITION` 且 `flags & 0x09 == 0x09` 时接受目标。
3. 初期可以只更新 pending/last target，不立即启动电机。
4. 确认通信稳定后，再允许小幅位置执行。

## 5. G4 返回给 H7 的状态帧

G4 发送：

```text
ID:      0x180
Format:  Standard ID
Frame:   CAN FD + BRS
DLC:     32 bytes
```

payload：

```text
byte0      protocol_version
byte1      board_id
byte2      online
byte3      control_word
byte4..5   rx_count low16 little-endian
byte6..7   tx_count low16 little-endian
byte8..11  error_count uint32 little-endian
byte12     pwm_started_mask
byte13     last_relative_seq
byte14     last_relative_seq_valid
byte15     position_pending
byte16     last_position_seq
byte17     last_position_flags
byte18     last_position_motor_mask
byte19     last_position_command
byte20..31 reserved
```

建议 G4 周期发送 0x180：

```text
10Hz ~ 100Hz 均可
第一版建议 20Hz 或 50Hz
```

如果 G4 暂时还没有周期状态任务，也可以在每次收到 0x120 后立即回一帧 0x180。

## 6. H7 串口测试命令

H7 通过 USART1 接收文本命令。

建议使用 VOFA 或串口助手发送 `Abc` 字符串，并带 `\r\n`：

```text
CANSTAT\r\n
CANSTOP\r\n
CANPOS 0 0 0\r\n
CANPOS 1000 0 0\r\n
```

### 6.1 CANSTAT

查询 H7 当前 CAN 状态和最近一次 G4 状态。

预期返回示例：

```text
CAN ready=1 tx=0 g4_valid=1 online=1 age=12 rx=25 g4rx=3 g4tx=25 err=0 last_seq=2 cmd=1 flags=9 mask=7 pending=0 pwm=7
```

关键字段：

```text
ready=1      H7 FDCAN app 初始化成功
g4_valid=1   H7 已经收到过 G4 0x180
online=1     G4 声明在线
age          距离最近一次收到 0x180 的时间
rx           H7 收到 0x180 的次数
g4rx         G4 收到 0x120 的计数 low16
err          G4 错误计数
last_seq     G4 最近收到的位置命令 seq
cmd          G4 最近收到的位置命令 command
flags        G4 最近收到的位置命令 flags
mask         G4 最近收到的位置命令 motor_mask
```

### 6.2 CANSTOP

H7 发送 stop 帧。

预期返回：

```text
OK CANSTOP status=0 tx=1
```

若失败：

```text
ERR CANSTOP status=x
```

### 6.3 CANPOS

H7 发送三电机目标位置定点值。

示例：

```text
CANPOS 0 0 0
CANPOS 1000 0 0
```

预期返回：

```text
OK CANPOS 1000 0 0 status=0 tx=2
```

若失败：

```text
ERR CANPOS status=x
```

## 7. 推荐测试顺序

### 7.1 只看 G4 状态

先不要发位置命令，只让 G4 周期发 0x180。

H7 串口发送：

```text
CANSTAT
```

期望：

```text
ready=1
g4_valid=1
online=1
rx 持续增加
```

### 7.2 发 STOP

H7 串口发送：

```text
CANSTOP
```

然后发送：

```text
CANSTAT
```

期望 G4 的 `g4rx` 增加。

### 7.3 发零目标

H7 串口发送：

```text
CANPOS 0 0 0
```

然后发送：

```text
CANSTAT
```

期望：

```text
last_position_command = 1
last_position_flags = 9
last_position_motor_mask = 7
error_count 不增加
```

### 7.4 发小目标

确认电机安全后，发送很小目标：

```text
CANPOS 1000 0 0
```

含义是 motor0 目标约 `0.01 rad`。

第一版不要直接发送大目标。

## 8. 闭环通过标准

满足以下条件即可认为通信闭环打通：

```text
H7 CANSTAT 显示 ready=1
H7 CANSTAT 显示 g4_valid=1
H7 能看到 G4 0x180 rx 计数增加
H7 发送 CANSTOP 后 G4 rx_count 增加
H7 发送 CANPOS 后 G4 last_position_seq / cmd / flags / mask 更新
G4 error_count 不持续增加
```

## 9. 常见问题

### 9.1 H7 ready=0

说明 H7 的 `Can_App_Init()` 未成功。

检查：

```text
FDCAN CubeMX 配置
FDCAN 时钟
Message RAM
是否调用 Can_App_Init()
```

### 9.2 g4_valid=0

说明 H7 没有收到 G4 的 0x180。

检查：

```text
G4 是否发送 0x180
H7/G4 波特率是否一致
CAN FD/BRS 是否一致
收发器是否支持 CAN FD 5M
终端电阻和接线
```

### 9.3 G4 收不到 0x120

检查：

```text
H7 是否执行 CANPOS/CANSTOP
H7 tx 是否增加
G4 filter 是否接收 0x120
G4 Message RAM element size 是否为 32 bytes
G4 是否打开 FDCAN Start 和 RX 中断
```

### 9.4 error_count 持续增加

优先检查：

```text
Nominal 1M / Data 5M 是否一致
BRS 是否一致
GPIO speed
CAN FD 收发器能力
总线终端电阻
H7/G4 是否共地
```
