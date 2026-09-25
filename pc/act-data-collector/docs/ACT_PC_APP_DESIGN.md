# 柔性触手 ACT 采集上位机设计方案

## 1. 目标与边界

本上位机专门用于单触手 ACT 示教控制、数据采集、Episode 保存和通信诊断，不替代现有四触手控制上位机。

两个串口职责固定：

```text
UART7：原有 VOFA/四触手控制
USART1：ACT 单触手控制与采样数据
```

第一版目标是稳定采集完整 Episode。训练和推理不放入上位机主界面，也不同时控制多条触手。

## 2. 工程路径

```text
F:\act_vla\flexible_tentacle_act
├─ pc_collector
│  ├─ main.py
│  ├─ app.py
│  ├─ serial_service.py
│  ├─ protocol.py
│  ├─ episode_buffer.py
│  ├─ tentacle_view.py
│  ├─ tof_heatmap.py
│  └─ requirements.txt
├─ datasets
│  ├─ raw
│  └─ processed
├─ training
│  ├─ build_dataset.py
│  ├─ train_act.py
│  ├─ evaluate_act.py
│  └─ export_onnx.py
├─ models
│  ├─ checkpoints
│  └─ exported
├─ configs
├─ docs
│  └─ ACT_PC_APP_DESIGN.md
└─ README.md
```

ACT 固件工程：

```text
D:\STM32Cube_project\4.8\H743VIT6_ACT
```

## 3. 与原上位机的关系

参考工程：

```text
D:\STM32Cube_project\4.8\PC_Tentacle_Controller
```

可以复用：

| 原模块 | 复用内容 | ACT 上位机调整 |
|---|---|---|
| `main.py` | PySide6 入口和样式 | 启动 ACT 采集窗口 |
| `serial_service.py` | COM 枚举、连接和重连 | USART1 文本发送、二进制接收 |
| `tentacle_view.py` | 触手姿态绘制 | 只显示当前单触手 |
| `app.py` | 控件样式和布局 | 改为单触手示教与采集流程 |

不复用四触手模式、`SNAP?/POS?` 轮询、VOFA 命令组、按键模式和 PC/AI 控制源页面。

## 4. 通信总体结构

通信采用简单的混合方式：

```text
PC -> H7：以换行结束的 ASCII 文本命令
H7 -> PC：带长度和 CRC32 的二进制数据包
```

这样做的原因：

- H7 接收命令容易调试，不需要复杂的二进制命令解析器。
- H7 返回的状态和 ACT_FRAME 数据量较大，二进制传输更高效。
- UART7 与 USART1 完全分开，不会混入 VOFA 文本。

## 5. 当前已经实现的第一阶段协议

### 5.1 当前命令

```text
ACT PING
ACT SELECT 1
ACT SELECT 2
ACT SELECT 3
ACT SELECT 4
ACT STATUS
```

每条命令必须以 `\n` 或 `\r\n` 结束。当前 H7 已实现单触手控制、采集控制和 ACT_FRAME 输出。

### 5.2 H7 二进制包格式

```text
magic        uint16   固定 0xA55A
type         uint8    数据包类型
version      uint8    当前为 1
data_length  uint16   有效数据字节数
data         uint8[]  有效数据
crc32        uint32   对前面全部字节计算的 CRC32
```

多字节字段均使用小端顺序。CRC32 参数：

```text
初值：0xFFFFFFFF
反射多项式：0xEDB88320
最终异或：0xFFFFFFFF
```

当前包类型：

```text
1：ACK
2：STATUS
3：ERROR
4：FRAME
```

### 5.3 ACK 与 ERROR 数据

```text
data[0]：command_id
data[1]：result
```

命令编号：

```text
0：UNKNOWN
1：PING
2：SELECT
3：STATUS
4：ENABLE
5：MOVE
6：HOME
7：STOP
8：CAPTURE
```

结果编号：

```text
0：成功
1：命令格式错误
2：参数错误
```

`ACT STATUS` 直接返回 STATUS 包，不再额外返回 ACK。

### 5.4 STATUS 数据

```text
selected_tentacle   uint8
control_enabled     uint8
capture_state       uint8
reserved            uint8
sent_frame_count    uint32
dropped_frame_count uint32
```

这些字段均由当前固件实时更新；`sent_frame_count` 和 `dropped_frame_count` 在一次新采集开始时清零。

### 5.5 当前请求处理规则

当前 H7 只有一个小型 TX DMA 缓冲区，因此上位机必须：

1. 发送一条状态命令。
2. 等待完整响应并校验 CRC32。
3. 收到响应或超时后，再发送下一条状态命令。

当前不设置通用命令序号，也不能连续堆积多条状态命令。ACT_FRAME 使用 VL53 的全局 `frame_seq` 检测采样丢帧。

## 6. 后续控制命令

在第一阶段通信验证通过后，按以下形式扩展：

```text
ACT ENABLE 1
ACT ENABLE 0
ACT MOVE x y bend stiff
ACT HOME
ACT STOP
ACT CAPTURE 1
ACT CAPTURE 0
ACT STATUS
```

处理规则：

- `ENABLE/SELECT/HOME/STOP/CAPTURE` 是状态命令，每次执行只返回一次 ACK 或 ERROR。
- `MOVE` 最高 20 Hz，只保留最新目标，不排队旧目标，也不为每一帧返回 ACK。
- `STOP` 始终具有最高控制优先级。
- `CAPTURE 1` 只开启 ACT_FRAME 数据流，不自动启动电机。
- `CAPTURE 0` 只停止 ACT_FRAME 数据流，不自动停止电机。

## 7. 上位机功能

### 7.1 串口连接

界面显示：

```text
COM 口选择与刷新
连接/断开
协议同步状态
RX/TX 字节数
CRC 错误数
frame_seq 丢帧数
H7 丢帧计数
```

USART1 当前固定使用 `2 Mbps, 8N1`；上位机将其设为默认值，其他波特率只用于明确的旧版小包调试。

连接后依次发送：

```text
ACT PING
ACT STATUS
```

只有收到合法响应后才显示“H7 已同步”。打开 COM 成功不代表协议已经同步。

### 7.2 单触手选择与控制

选择范围为触手 1～4。切换触手时必须处于未采样、未启用控制的状态。

示教参数沿用 H7 单触手映射语义：

```text
X：-100～100
Y：-100～100
Bend：0～100
Stiff：0～100
```

界面高频变化只更新本地“最新目标”。串口发送定时器最多以 20 Hz 发送一次最新 `ACT MOVE`，不能累计历史目标。

### 7.3 姿态和热力图

触手视图显示当前触手编号、方向、弯曲程度和 `actual_q[3]`。

8×8 热力图用于检查瓶子位置、无效像素和触手遮挡。热力图可以按 5～10 Hz 降频刷新，但原始 ACT_FRAME 必须完整进入 EpisodeBuffer。

## 8. 一帧 ACT 原始数据

H7 每产生一帧数据，就把同一采样时刻附近的 ToF 数据和电机实际位置放进同一个 ACT_FRAME：

```text
frame_seq                  VL53 全局有效帧编号
timestamp_ms               ToF 帧读取完成时间
tentacle                   当前示教触手编号

distance_mm[64]            8×8 距离
target_status[64]          测距状态码
target_count[64]           每个区域检测到的目标数量
signal[64]                 目标回波强度
ambient[64]                环境红外强度
reflectance[64]            反射率
sigma_mm[64]               距离不确定度
cnh_hist[64][10]           10 个固定距离区间的 CNH 直方图
cnh_scaler[64][10]         对应 CNH 缩放参数

actual_q[3]                当前三台电机相对 HOME 的实际位置
action_q[3]                最近成功进入 CAN 队列的三电机目标
valid_flags                actual/action有效位及FAULT位
status_age_ms              最近CAN状态年龄
```

坐标语义统一为：

```text
正数：收线
负数：放线
1 q：0.01 rad 电机机械角
```

`frame_seq` 不随 `ACT CAPTURE 1` 清零。上位机以本次采集首帧为基准记录缺口，不能假设编号从 1 开始。

上位机原样保存当前固件提供的 `actual_q`、`action_q`、`valid_flags` 和 `status_age_ms`。不由上位机伪造不存在的 `motor_rx_seq` 或 `motor_tick_ms`。

## 9. Episode 采集与保存

采集状态：

```text
空闲 -> 开始采样 -> 采样中 -> 停止采样 -> 等待保存
等待保存 -> 保存 Episode -> 空闲
等待保存 -> 删除 Episode -> 空闲
```

采样期间：

- 上位机逐帧接收并校验 ACT_FRAME。
- 合法帧立即追加到内存 Episode。
- 不为每一帧创建文件，也不在采样期间写正式数据集。
- `sample_seq` 发生跳变时记录缺口，后续不得跨缺口构造训练样本。

用户确认保存后一次性生成：

```text
datasets\raw\episode_000001
├─ meta.json
└─ raw_data.npz
```

`raw_data.npz` 数组：

```text
frame_seq        [N]
timestamp_ms     [N]
pc_receive_ns    [N]
tentacle         [N]
distance_mm      [N,64]
target_status    [N,64]
target_count     [N,64]
signal_per_spad  [N,64]
ambient_per_spad [N,64]
reflectance      [N,64]
range_sigma_mm   [N,64]
cnh_raw          [N,10,64]
cnh_scaler       [N,10,64]
actual_q         [N,3]
action_q         [N,3]
valid_flags      [N]
status_age_ms    [N]
```

保存在线程中执行，先写临时文件，全部成功后再改为正式名称。删除操作只清空尚未保存的内存 Episode。

`N` 是本次实际收到的完整帧数，不固定。固定的只是单个 ACT_FRAME 数据布局；
采集可由用户在任意时刻停止，所有数组第一维随 Episode 时长变化。保存时同时在
`meta.json.training_compatibility` 写入字段/尺寸校验结果和有效训练窗口数量。

## 10. 训练数据构造

### 10.1 单帧模型输入

对 Episode 中第 `k` 帧构造：

```text
tof[k]    [18,8,8]
actual[k] [3]
```

18 个 ToF 通道：

```text
0       distance
1       valid_mask
2       status_ok
3       target_count
4       signal
5       ambient
6       reflectance
7       sigma
8～17   CNH bin 0～9
```

`valid_mask` 和 `status_ok` 在离线预处理中根据原始距离、状态码和设定范围计算。CNH 使用固定的 10 个距离区间，不为每个 Episode 单独采集背景，也不把背景图作为额外模型输入。

模型输入不包含过去 12 帧，也不包含操作者目标命令。当前电机位置 `actual[k]` 用来告诉模型触手此刻已经运动到哪里。

### 10.2 监督标签

第 `k` 帧的标签是后续 12 帧真实电机位置：

```text
future_actual[k] = actual[k+1], ..., actual[k+12]
shape = [12,3]
```

采样频率为 20 Hz 时，标签覆盖未来约 0.6 秒。标签来自示教过程中真实记录的后续位置，不是当前控制命令。

以下位置不能构造样本：

- `sample_seq` 缺口两侧跨越的位置。
- Episode 末尾不足 12 个未来帧的位置。
- ToF 与电机时间差超过最终设定阈值的位置。

## 11. 上位机模块职责

### `serial_service.py`

负责 COM 枚举、连接、断开、原始字节接收和文本命令发送，不解析业务数据。

### `protocol.py`

负责生成带换行的 ACT 文本命令，以及对 H7 二进制字节流执行：

```text
查找 magic
读取固定包头
等待完整 data_length
检查 version 和长度
校验 CRC32
输出完整消息
错误后重新寻找下一个 magic
```

一次 `readyRead` 可能只有半包，也可能包含多个包，不能假设一次回调等于一个数据包。

### `episode_buffer.py`

负责内存 Episode、`frame_seq` 连续性、保存和删除，不直接操作串口或电机。

### `app.py`

负责界面状态转换和按钮使能，不直接解析协议，也不直接写 NPZ。

### `tentacle_view.py` 与上位机热力图控件

只负责降频显示，不能修改或丢弃 EpisodeBuffer 中的原始数据。

## 12. H7 完整数据流需要的发送结构

当前 H7 使用一个大 ACT_FRAME DMA 缓冲区和小包等待槽：

```text
一个大 ACT_FRAME 缓冲区（忙时保留最新传感器帧并累计跳过数量）
一个小型 ACK/STATUS/ERROR 等待槽
一个函数统一拥有 HAL_UART_Transmit_DMA
```

发送原则：

- 正在发送的缓冲区绝不允许被覆盖。
- 下一帧缓冲区已占用时，丢弃整帧并增加 `dropped_frame_count`。
- 不能发送半帧，也不能阻塞 CAN、VL53 或电机任务等待串口。
- 当前大帧发送完成后，优先发送等待中的 ACK/ERROR。
- TX 缓冲区按 32 字节对齐，DMA 前清理 DCache。
- RX 循环 DMA 缓冲区按 32 字节对齐，CPU 读取前失效 DCache。

在 2 Mbps、8N1 下，理论有效字节率约 200 KB/s。约 4.2 KB 的 ACT_FRAME 以 20 Hz 发送约占 84 KB/s，带宽有余量。

## 13. 开发与验收顺序

### 第一阶段：通信底座

已完成 H7 端：

```text
USART1 循环 RX DMA
ACT PING
ACT SELECT 1～4
ACT STATUS
ACK/STATUS/ERROR 二进制包
CRC32
TX 超时和 UART 错误恢复
```

PC 最小串口测试和 1000 次分片 PING 解析回归已经完成。

### 第二阶段：单触手控制

接入 `ENABLE/MOVE/HOME/STOP`，验证方向、限幅和 STOP 行为与 UART7 的 SOLO 控制一致。

### 第三阶段：ACT_FRAME

USART1 使用 2 Mbps，接入20 Hz完整帧、电机/ToF对齐和EpisodeBuffer。连续采集15秒应得到约300帧。

### 第四阶段：数据保存与训练

上位机完成 `meta.json` 与 `raw_data.npz` 原始Episode保存；后续继续完成数据清洗、未来12帧标签构造、训练、评估和ONNX导出。

## 14. 第一版不做的功能

```text
四触手同时示教
串口7协议兼容层
SD卡ACT采集
每帧独立文件
上位机实时训练
上位机实时ACT推理
自动判断抓取成功
额外背景图输入
复杂三维重建
```

第一版只解决一件事：稳定控制一条触手，并可靠保存可用于 ACT 训练的完整 Episode。
