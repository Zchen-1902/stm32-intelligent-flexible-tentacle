# H743 手势识别采集项目进度文档

## 0. 文档状态说明

本文前半部分保留了项目早期推进记录，因此其中有些“当前未完成项”已经属于历史状态。`2026-07-05` 之前的复盘见第 12 章；之后的问题、解决过程和当前风险按时间顺序续写在第 13 章。

## 1. 任务目标

当前目标是把 `STM32H743VIT6` 开发板作为手势主控板使用，围绕 `VL53L8CH / VL53L8CH` ToF 传感器完成静态手部张握程度数据采集，并为后续训练、CubeAI 转换、量化部署和 CANFD 控制通讯做准备。

第一阶段的重点不是直接完成最终推理和控制，而是先稳定打通以下链路：

1. `VL53` 传感器初始化和 8x8 距离矩阵读取。
2. `CNH` 直方图特征输出和解析。
3. SD 卡 CSV 数据保存。
4. USART1 串口调试输出和 VOFA 命令控制。
5. LED 状态反馈。
6. 为后续训练数据打标签，最终得到 `0~100` 的手部张握程度回归值。

目标输出不是“识别具体手势类别”，而是输出一个连续或准连续的张握程度分数：

```text
0   = 接近握拳
25  = 偏握拳
50  = 半握/中间状态
75  = 偏张开
100 = 接近张开
255 = 无手/无效样本标签
```

后续希望通过 STM32CubeAI 将训练好的模型转换、量化并部署到 H743 上运行。

## 2. 当前总体方案

当前方案分为四层：

### 2.1 感知层

感知层由 `VL53_App` 负责，主要功能包括：

- 初始化 VL53 传感器。
- 配置 8x8 分辨率。
- 配置 CNH 输出。
- 每帧读取距离矩阵 `distance[64]`。
- 每帧读取 CNH 特征 `cnh[64][8]`。
- 对外提供采样请求接口。
- 在采样模式下将当前稳定帧保存到 SD 卡 CSV。

当前 CNH 参数为：

```c
#define VL53_APP_CNH_START_BIN   6
#define VL53_APP_CNH_NUM_BINS    8
#define VL53_APP_CNH_SUB_SAMPLE  1
```

含义：

- `START_BIN = 6`：从第 6 个距离 bin 开始取直方图。
- `NUM_BINS = 8`：每个 zone 保存 8 个 bin。
- `SUB_SAMPLE = 1`：不做额外降采样。

当前 CSV 特征规模：

```text
4 个基础字段 + 64 个距离值 + 64 * 8 个 CNH 值 = 580 列
```

当前 CSV 字段为：

```csv
timestamp_ms,frame_count,mode,valid,d0...d63,cnh_z0_b0...cnh_z63_b7
```

注意：当前 `label_score` 还没有正式写入 CSV，这是下一步必须补上的内容。

### 2.2 采集层

采集层现在仍由 `VL53_App` 内部完成具体写文件动作。

采样流程为：

1. 上层调用 `VL53_App_RequestSample()`。
2. VL53 模块记录当前请求时间。
3. 等待约 `500 ms`，让手部姿态稳定。
4. 将当前最新帧整理为 CSV 行。
5. 调用 `CsvLogger_AppendLine()` 写入 `0:/gesture.csv`。
6. 写入完成后通过 `VL53_App_ConsumeSampleResult()` 给上层返回结果。

当前 SD 写入策略：

- 每次采样都 `open -> write -> sync -> close`。
- 文件为空时自动写入 header。
- 每行写完后 `f_sync()`。
- 每次写完关闭文件，降低突然断电导致文件损坏的风险。

这样速度不是最高，但对当前人工采样和训练数据采集更稳。

### 2.3 VOFA 控制层

已经新增独立模块：

```text
H743VIT6/Core/Vofa_debug/vofa.h
H743VIT6/Core/Vofa_debug/vofa.c
```

VOFA 模块职责：

- 使用 USART1 DMA 环形缓冲接收字符串命令。
- 按 `\r` 或 `\n` 判断一条命令结束。
- 在主循环中解析命令，避免中断中做复杂操作。
- 控制采样模式、单次采样、连续采样和 LED 状态。
- 提供 `STATUS`、`HELP`、`ERRCLR` 等调试命令。

当前 USART1 作为 VOFA 控制串口，使用字符串 `Abc` 模式发送命令。

VOFA 按钮建议发送内容：

```text
MODE\r\n
REC\r\n
SAMPLE\r\n
STOP\r\n
SETREC 30 500\r\n
LABEL 0\r\n
LABEL 25\r\n
LABEL 50\r\n
LABEL 75\r\n
LABEL 100\r\n
LABEL 255\r\n
STATUS\r\n
ERRCLR\r\n
HELP\r\n
```

当前命令含义：

| 命令 | 作用 |
|---|---|
| `MODE` | 在推理模式和采样模式之间反转 |
| `REC` | 连续采样开关，未开启则开始，已开启则停止 |
| `SAMPLE` | 单次采样一次 |
| `STOP` | 停止连续采样 |
| `SETREC n ms` | 设置连续采样次数和采样间隔 |
| `LABEL x` | 设置当前标签，`0~100` 或 `255` |
| `STATUS` | 输出当前模式、采样状态、连续采样配置、标签和错误状态 |
| `ERRCLR` | 清除错误标志和错误灯 |
| `HELP` | 输出命令帮助 |

当前默认连续采样参数：

```c
#define VOFA_DEFAULT_REC_COUNT  30U
#define VOFA_DEFAULT_REC_MS     500U
#define VOFA_LABEL_NONE         255U
```

### 2.4 LED 状态反馈

当前 LED 约定如下：

| LED | 含义 | 状态 |
|---|---|---|
| LED1 | 当前处于采样模式 | 亮 = SAMPLE，灭 = INFER |
| LED2 | 当前有采样请求正在等待写入完成 | 亮 = 正在采样/写入 |
| LED3 | 错误状态 | 亮 = 最近存在错误 |

LED 采用低电平点亮：

```c
#define VOFA_LED_ON   GPIO_PIN_RESET
#define VOFA_LED_OFF  GPIO_PIN_SET
```

## 3. 遇到的问题与解决过程

### 3.1 串口没有数据输出

问题表现：

- 一开始 USART1 没有调试输出。
- 使用官方/开发板例程测试串口通信正常，说明硬件本身没有问题。
- 后续 Debug 发现程序会卡在初始化流程中，导致串口打印没有机会执行。

原因分析：

- 程序初始化链路中包含 SD / FatFs 等外设。
- 当 SD 卡未插入或初始化异常时，可能进入错误路径。
- 这会导致后续 VL53 和串口调试逻辑没有继续运行。

解决方式：

- 暂时绕开会卡死的 SD 初始化问题。
- 后续恢复 SD 采样时，避免未插卡直接导致整体程序不可观察。
- 加入 LED 和串口状态输出，方便判断程序是否跑起来。

结果：

- USART1 能输出 VL53 调试信息。
- 可以通过串口看到传感器帧数据和 CSV 写入状态。

### 3.2 VL53 距离值显示异常，近似放大或比例错误

问题表现：

- 实际距离约 `20 cm`，显示却接近 `1000 mm`。
- 实际 `30 cm` 左右，显示约 `1500 mm`。
- 表现为数值明显按比例异常。

原因分析：

- VL53 ULD API 的输出格式配置与当前读取方式不匹配。
- 原始格式和普通 `distance_mm` 解释方式混用，导致距离值被错误解释。

解决方式：

- 关闭 `VL53LMZ_USE_RAW_FORMAT`。
- 让 `results->distance_mm[]` 直接按普通 mm 单位读取。

结果：

- 距离显示恢复准确。
- 后续 CSV 中的 `d0...d63` 可以作为真实距离输入特征使用。

### 3.3 CNH 如何理解和使用不清楚

问题表现：

- 对 `bin`、`直方图`、`回波`、`CNH` 的物理意义不清楚。
- 一开始容易把 CNH 当成普通距离值。

解释结果：

- VL53 发射光脉冲，物体反射回来形成回波。
- 回波到达时间对应距离。
- CNH 是把不同距离段内的回波强度整理成直方图。
- 每个 bin 表示一个距离段中的回波强度。
- 对于 8x8，每个 zone 都有一组 CNH bin。

当前用途：

- 距离矩阵 `distance[64]` 更适合定位手的位置和移动趋势。
- CNH `cnh[64][8]` 更适合表达手部形态、体积和反射分布，用于静态张握程度回归。

### 3.4 采样按钮不适合长期使用

问题表现：

- 使用实体按键触发采样太费手。
- 长期大量采样可能损坏按键。
- 按键操作也容易引入手部动作干扰。

解决方式：

- 改为通过 VOFA 串口命令控制采样。
- 单次采样、连续采样、模式切换、标签设置都由 VOFA 按钮发送字符串命令完成。

结果：

- 不再依赖实体按键。
- 可以用 VOFA 建多个按钮快速采集不同标签的数据。
- 连续采样可以指定次数和间隔，减少人工操作。

### 3.5 VOFA 只能发送字符串或十六进制

问题表现：

- VOFA 命令控件只能选择 `Abc` 字符串或 `Hex` 十六进制。

解决方式：

- 选择 `Abc` 字符串模式。
- 每条命令以 `\r\n` 结束。
- MCU 端以 `\r` 或 `\n` 作为命令结束符。

结果：

- 命令形式简单清晰。
- 不需要使用复杂二进制协议。
- 适合当前采样阶段。

### 3.6 SD CSV 保存可靠性问题

问题表现：

- 需要确认每次采样是否真的写入 SD。
- 需要避免文件没有 header 或列数不一致。
- 需要避免长时间打开文件导致断电风险。

解决方式：

- `CsvLogger_AppendLine()` 内部每次打开文件。
- 文件为空时写 header。
- 写数据行后 `f_sync()`。
- 最后 `f_close()`。
- 写入失败返回错误码并通过串口输出。

结果：

- CSV 数据可以持续写入。
- 每行列数固定。
- 断电风险比长期保持文件打开更低。

## 4. 当前已经完成的内容

### 4.1 VL53 基础测距

已完成：

- VL53 初始化。
- 8x8 分辨率输出。
- 距离矩阵读取。
- 距离单位修正为真实 mm。
- 中心 zone 串口调试输出。

当前输出示例语义：

```text
frame_count    当前读取帧计数
timestamp_ms   HAL tick 时间戳
z27 d=xxx      中心 zone 的距离 mm
st=xxx         target status
cnh_valid      CNH 是否成功提取
```

### 4.2 CNH 输出

已完成：

- CNH 配置。
- 创建 8x8 aggregate map。
- 通过自定义 output config 追加 CNH block。
- 使用 `vl53lmz_send_output_config_and_start()` 启动带 CNH 的 ranging。
- 使用 `vl53lmz_dci_read_data()` 提取 CNH block。
- 每个 zone 提取 8 个 bin。

当前 CNH 数据规模：

```text
64 zones * 8 bins = 512 个 CNH 特征
```

### 4.3 CSV 写入

已完成：

- 写入 `0:/gesture.csv`。
- 自动写 header。
- 写入 64 个距离值。
- 写入 512 个 CNH 值。
- 每次采样单独打开、同步、关闭文件。
- 写入结果通过 `VL53_App_ConsumeSampleResult()` 给 VOFA 层消费。

### 4.4 VOFA 控制模块

已完成：

- 新增独立目录：`Core/Vofa_debug`。
- 新增 `vofa.c` / `vofa.h`。
- 使用 USART1 DMA 环形缓冲接收字符串。
- 支持 `MODE`、`REC`、`SAMPLE`、`STOP`、`SETREC`、`LABEL`、`STATUS`、`ERRCLR`、`HELP`。
- `main.c` 中只调用：

```c
Vofa_Init();
Vofa_Task();
```

### 4.5 LED 状态

已完成：

- LED1 显示采样模式。
- LED2 显示采样等待/写入中。
- LED3 显示错误状态。

## 5. 当前解决方案的数据流

当前运行主流程：

```text
main()
  HAL_Init()
  SystemClock_Config()
  MX_GPIO_Init()
  MX_DMA_Init()
  MX_USART1_UART_Init()
  MX_I2C1_Init()
  MX_SDMMC1_SD_Init()
  MX_FATFS_Init()
  ...
  App_LedInit()
  VL53_App_Init()
  VL53_App_SetMode(INFER)
  Vofa_Init()

while(1)
  VL53_App_Task()
    check_data_ready
    get_ranging_data
    extract CNH block
    copy distance frame
    if sample pending and delay >= 500ms:
      build csv line
      append csv line to SD
      set sample result pending

  Vofa_Task()
    receive UART command line
    parse command
    update mode / label / record state
    consume sample result
    schedule next sample if REC active
```

当前采样流程：

```text
VOFA: LABEL 50\r\n
  -> MCU 保存当前 label_score = 50

VOFA: REC\r\n
  -> 开始连续采样
  -> 使用默认或 SETREC 设置的次数/间隔
  -> 每次触发 VL53_App_RequestSample()
  -> VL53 内部延时 500ms 后写 CSV
  -> 写完后 VOFA 层收到结果
  -> 成功则等待下一个 interval
  -> 次数耗尽则 OK REC DONE
```

当前单次采样流程：

```text
VOFA: SAMPLE\r\n
  -> 停止连续采样状态
  -> 进入 SAMPLE 模式
  -> 请求单次采样
  -> LED2 亮
  -> 写入完成后 LED2 灭
  -> 成功 LED3 灭，失败 LED3 亮
```

## 6. 当前仍存在的问题

### 6.1 `LABEL` 尚未写入 CSV

这是当前最重要的未完成项。

现在 `vofa.c` 内部已经保存了：

```c
g_state.label_score
```

但 `vl53_app.c` 写 CSV 时并不知道这个 label，所以当前 CSV 中还没有 `label_score` 字段。

当前 CSV：

```csv
timestamp_ms,frame_count,mode,valid,d0...d63,cnh...
```

目标 CSV 应改为：

```csv
timestamp_ms,label_score,frame_count,mode,valid,d0...d63,cnh...
```

或者：

```csv
timestamp_ms,frame_count,label_score,mode,valid,d0...d63,cnh...
```

建议下一步采用第一种，因为训练脚本更容易读：

```csv
timestamp_ms,label_score,frame_count,mode,valid,...
```

需要新增接口，让 VOFA 设置的 label 能被 VL53 CSV 写入使用。

建议最小修改：

```c
void VL53_App_SetLabelScore(uint8_t label_score);
uint8_t VL53_App_GetLabelScore(void);
```

然后 `vofa.c` 中执行 `LABEL x` 时同时调用：

```c
VL53_App_SetLabelScore((uint8_t)label);
```

### 6.2 当前没有离线训练脚本

还没有建立 PC 端训练流程。

后续需要：

1. 读取 `gesture.csv`。
2. 检查列数。
3. 分离 label、distance、CNH。
4. 做归一化。
5. 划分训练集/验证集。
6. 训练回归模型。
7. 导出 ONNX / TFLite / CubeAI 支持格式。
8. 使用 STM32CubeAI 做量化和代码生成。

### 6.3 当前未实现推理模式

`MODE INFER` 目前只是关闭采样写 SD，尚未真正运行模型推理。

后续推理模式应完成：

- 读取最新 VL53 frame。
- 组装模型输入。
- 调用 CubeAI 网络推理。
- 得到 `openness_score = 0~100`。
- 做滤波。
- 输出给控制逻辑。
- 最终通过 CANFD 发给电机板。

### 6.4 当前没有 CANFD 控制摘要发送

当前 CANFD 最终目标尚未实现。

目标是 H743 向电机板发送控制摘要，不发送原始 CNH。

建议 payload 包含：

```text
magic
version
seq
valid
openness_score
center_x
center_y
center_z_mm
motion_x
motion_y
motion_z
state
flags
crc
```

### 6.5 当前采样间隔语义需要进一步确认

当前 `REC` 逻辑是：

1. 发送 `REC` 后立刻触发第一帧采样。
2. VL53 内部等待 500ms 后写入。
3. 写入成功后等待 `record_interval_ms`。
4. 再触发下一帧。

所以实际相邻 CSV 行之间的时间大约是：

```text
500ms 稳定等待 + record_interval_ms + 文件写入时间
```

如果希望 `SETREC 30 500` 表示“CSV 行之间约 500ms”，则需要改调度语义。

当前更准确的理解是：

```text
每次写入前先稳定 500ms，写完后再等 interval_ms 再发下一次采样请求。
```

### 6.6 当前 SD 写入是可靠优先，不是速度优先

每次写入都 open/sync/close，稳定但较慢。

如果后续需要高频采样，可能要改为：

- 采样开始时打开文件。
- 连续采样期间保持文件打开。
- 每 N 行或每 1 秒 sync 一次。
- 采样结束后 close。

但当前阶段不建议马上改，因为可靠性和简单性更重要。

### 6.7 当前 VOFA 打印是阻塞式

`Vofa_Printf()` 使用：

```c
HAL_UART_Transmit(..., HAL_MAX_DELAY)
```

当前采样频率较低，问题不大。

如果后续进入实时控制和 CANFD 高频运行，需要改为：

- 降低打印频率。
- 或使用 UART DMA TX 队列。
- 或最终关闭调试打印。

### 6.8 当前没有完整错误恢复策略

现在错误主要表现为 LED3 亮和串口返回 `ERR xxx`。

还没有完整区分：

- SD 未插卡。
- SD mount 失败。
- 文件打开失败。
- CNH 提取失败。
- VL53 通信失败。
- UART RX 异常。

后续可以把错误码进一步标准化。

## 7. 后续优化建议

### 7.1 优先补齐 label 写入 CSV

这是下一步最高优先级。

原因：

- 没有 label 的 CSV 不能直接用于监督训练。
- 当前 VOFA 已经能设置 label，只差接到 CSV。

建议目标 CSV：

```csv
timestamp_ms,label_score,frame_count,mode,valid,d0...d63,cnh_z0_b0...cnh_z63_b7
```

其中：

- `label_score = 0~100` 表示有效手势张握程度。
- `label_score = 255` 表示无手数据。
- `valid` 表示当前帧目标是否有效。

### 7.2 增加 CSV 文件元信息

可以在 CSV 文件开头或单独 `meta.txt` 记录：

```text
sensor=VL53L8CH
resolution=8x8
cnh_start_bin=6
cnh_num_bins=8
cnh_sub_sample=1
sample_delay_ms=500
firmware_date=...
```

这样以后训练时不会忘记数据采集参数。

但第一版可以先不做，避免复杂化。

### 7.3 增加采样质量字段

后续可以在 CSV 中增加：

- 有效 zone 数量。
- 中心距离。
- 平均距离。
- 最近错误码。

这些字段可以帮助离线清洗数据。

但第一版训练不一定需要。

### 7.4 明确训练标签策略

建议第一批数据先采：

```text
LABEL 255  无手
LABEL 0    尽量握拳
LABEL 25   偏握拳
LABEL 50   中间状态
LABEL 75   偏张开
LABEL 100  尽量张开
```

每个标签多采不同距离、不同手型、不同位置：

- 250mm。
- 300mm。
- 350mm。
- 400mm。
- 450mm。
- 中心位置。
- 偏左、偏右、偏上、偏下。

这样比只在一个固定位置采更有泛化能力。

### 7.5 训练模型建议

第一版建议仍然采用小模型回归：

```text
input = distance[64] + cnh[64][8] = 576
output = openness_score 0~100
```

模型可以从小 MLP 开始：

```text
576 -> 16 -> 1
```

或者：

```text
576 -> 32 -> 8 -> 1
```

如果 CubeAI 量化后速度和 RAM 都够，再考虑加大模型。

### 7.6 推理输出滤波

即使模型输出连续值，也建议做滤波：

```text
score_filtered = score_filtered * 0.8 + score_new * 0.2
```

或者用整数滤波：

```c
filtered = (filtered * 4 + score_new) / 5;
```

这样可以减少手轻微抖动导致的控制抖动。

### 7.7 分离静态识别和动态检测

当前思路是合理的：

- 静态时使用 CNH + distance 做张握程度推理。
- 动态移动时主要使用 8x8 distance 做位置和速度检测。
- 不要求在快速移动过程中实时更新手势张握程度。

这样可以降低模型计算压力，也更符合应用需求。

## 8. 达到最终目标还需要完成的任务

### 阶段 1：补齐采集数据闭环

必须完成：

1. 将 `label_score` 写入 CSV。
2. 确认 VOFA `LABEL` 设置后，下一次 `SAMPLE/REC` 写入正确标签。
3. 采一小批数据并检查 CSV 列数、header、标签是否正确。
4. 确认无手 `255` 数据能被保存。

验收标准：

```text
gesture.csv 中每一行都有 label_score 字段。
不同 VOFA LABEL 按钮采到的数据标签正确。
CSV 行列数固定且可被 Python 读取。
```

### 阶段 2：批量采集训练数据

建议流程：

```text
MODE
SETREC 30 500
LABEL 255
REC
LABEL 0
REC
LABEL 25
REC
LABEL 50
REC
LABEL 75
REC
LABEL 100
REC
```

每个标签建议多轮采集，改变手的位置、距离和角度。

验收标准：

```text
每个标签至少有几百行可用数据。
CSV 中 valid、distance、CNH 数据合理。
无明显空行、断列、重复 header。
```

### 阶段 3：PC 端训练

需要实现：

1. CSV 读取脚本。
2. 数据清洗。
3. 特征归一化。
4. 回归模型训练。
5. 验证误差评估。
6. 导出 CubeAI 可转换模型。

验收标准：

```text
验证集上 0~100 输出趋势正确。
握拳接近 0，张开接近 100，中间状态输出合理。
无手数据能被识别或被 valid 逻辑过滤。
```

### 阶段 4：CubeAI 部署

需要完成：

1. 使用 STM32CubeAI 导入模型。
2. 配置量化。
3. 生成推理代码。
4. 在 H743 工程中接入 AI 初始化和推理。
5. 对比 PC 输出和 MCU 输出。

验收标准：

```text
同一组输入在 PC 和 MCU 上输出接近。
MCU 推理时间满足控制需求。
RAM/Flash 占用可接受。
```

### 阶段 5：控制逻辑与 CANFD 通讯

需要完成：

1. 从 8x8 distance 计算手部中心位置。
2. 计算移动方向和速度。
3. 融合 `openness_score + position + motion`。
4. 定义 CANFD payload。
5. H743 发送控制摘要。
6. 电机板接收并解析。
7. 异常状态下进入安全状态。

验收标准：

```text
H743 能稳定发送 0~100 张握程度和位置/运动摘要。
电机板能解析 CANFD 数据并做安全控制。
丢帧、无手、错误状态下不会输出危险动作。
```

## 9. 当前风险点

### 9.1 CubeMX 重新生成覆盖问题

当前 `vofa.c/h` 是用户新增模块，不属于 CubeMX 自动生成区。

风险较低。

但 `main.c` 中调用 `Vofa_Init()` 和 `Vofa_Task()` 必须放在 USER CODE 区域内，否则 CubeMX 重新生成可能丢失。

重新生成后必须检查：

```text
main.c 是否仍 include vofa.h
main.c USER CODE BEGIN 2 是否仍调用 Vofa_Init()
main.c USER CODE BEGIN 3 是否仍调用 Vofa_Task()
Keil 工程是否仍包含 vofa.c
IncludePath 是否包含 ../Core/Vofa_debug
```

### 9.2 Keil 工程文件改动较大

之前加入 `vofa.c` 到 Keil 工程时，`.uvprojx` 可能出现较大的格式变化。

后续提交前需要特别检查：

```text
H743VIT6/MDK-ARM/H743VIT6.uvprojx
```

确认只保留必要的新增 include path 和 `vofa.c` 文件加入，不要混入无关工程配置变化。

### 9.3 当前仓库中有大量历史未跟踪/修改文件

`git status` 显示当前工作区存在大量旧改动和未跟踪文件。

后续提交时必须小心，只提交本功能相关文件，避免把旧快照、node_modules、Keil 个人文件等一起提交。

建议提交范围：

```text
H743VIT6/Core/Vofa_debug/vofa.c
H743VIT6/Core/Vofa_debug/vofa.h
H743VIT6/Core/Src/main.c
H743VIT6/MDK-ARM/H743VIT6.uvprojx
H743VIT6/Core/vl53_app/vl53_app.c
H743VIT6/Core/vl53_app/vl53_app.h
H743VIT6/Core/Src/csv_logger.c
H743VIT6/Core/Inc/csv_logger.h
```

具体以每一步实际修改为准。

## 10. 下一步建议

下一步建议按最小修改继续：

1. 给 `VL53_App` 增加 `label_score` 状态和 setter/getter。
2. `vofa.c` 的 `LABEL x` 命令调用 `VL53_App_SetLabelScore(x)`。
3. 修改 CSV header 和 CSV 行，把 `label_score` 写入第二列。
4. 更新 `VL53_APP_CSV_COLUMNS`，从 `580` 变成 `581`。
5. 上板测试：

```text
LABEL 0
SAMPLE
LABEL 100
SAMPLE
STATUS
```

6. 拿 SD 卡查看 `gesture.csv`：

```csv
timestamp_ms,label_score,frame_count,mode,valid,...
```

确认两行标签分别是 `0` 和 `100`。

完成这个之后，才真正具备训练数据采集闭环。

## 11. 当前结论

当前项目已经从“传感器能否跑起来”推进到“可以开始组织训练数据采集”的阶段。

已经完成的关键基础：

- VL53 8x8 距离矩阵稳定输出。
- CNH 输出链路打通。
- 距离单位问题已修正。
- SD CSV 写入链路可用。
- VOFA 字符串命令控制框架已建立。
- 单次采样和连续采样状态机已具备雏形。
- LED 能反馈采样模式、采样等待和错误状态。

当前最关键的缺口：

```text
LABEL 已能通过 VOFA 设置，但还没有写入 CSV。
```

因此下一步不应直接开始大量采数据，而应先把标签写入 CSV。否则采出来的数据无法直接用于监督训练。

## 12. 2026-07-05 最新问题复盘与当前状态

本节记录从早期采集链路打通之后，到当前 H7 主控侧已经实际完成和修正的问题。这里的内容以当前代码状态为准。

### 12.1 当前 H7 的真实定位

当前 H7 已经不只是“采集板”，而是承担手势主控的完整上层任务：

```text
VL53L8CH 8x8 + CNH
  -> CSV 数据采集
  -> PC 端 CNN/MLP 训练
  -> CubeAI 部署推理
  -> openness_score 0~100
  -> 手部中心位置 / 距离 / 弯曲程度 / 刚度
  -> CANFD 下发给 G4 三电机板
  -> W25Q64 保存 HOME / ACTUAL 位置记录
```

当前 `main()` 主流程已经包含：

```c
Can_App_Init();
MotorControl_Init();
VL53_App_Init();
VL53_App_SetMode(VL53_APP_MODE_INFER);
Vofa_Init();

while (1)
{
  MX_X_CUBE_AI_Process();
  VL53_App_Task();
  Vofa_Task();
  MotorControl_Task();
}
```

说明：

- `VL53_App_Task()` 负责传感器帧、CNH、CSV、AI 前处理和推理。
- `Vofa_Task()` 负责串口命令、采样状态机、CAN/W25Q64 调试命令。
- `MotorControl_Task()` 负责把 AI 输出映射为触手控制目标，并通过 CANFD 发给 G4。

### 12.2 CSV 字段已经补齐，早期“label 未写入 CSV”已解决

早期文档中记录的主要缺口是 `LABEL` 尚未写入 CSV。当前已经解决。

当前 CSV 表头由 `VL53_App_BuildCsvHeader()` 生成，字段为：

```text
timestamp_ms,label_score,frame_count,mode,valid,
d0...d63,
target_status_z0...target_status_z63,
nb_target_z0...nb_target_z63,
cnh_z0_b0...cnh_z63_b7
```

当前列数：

```text
5 + 64 + 64 + 64 + 64*8 = 709 列
```

解决内容：

- `vofa.c` 的 `LABEL x` 命令会调用 `VL53_App_SetLabelScore()`。
- `VL53_App_WriteCsvFrame()` 会把 `g_vl53_label_score` 写入第二列。
- CSV 写入前会用 `VL53_App_CountCsvColumns()` 检查列数。
- 表头和数据行都按 `VL53_APP_CSV_COLUMNS` 校验，避免训练脚本读到错列数据。

当前结论：

```text
CSV 已经满足训练数据格式要求。
label_score、target_status、nb_target_detected、distance、CNH 均已进入数据集。
```

### 12.3 CSV 坏行、错行和文件损坏问题

问题表现：

- 早期采集得到的 CSV 中出现过错行、坏行。
- 有时电脑读取 SD 卡时提示根目录或文件损坏。
- VOFA 显示写入成功，但电脑端看不到完整文件或无法正常打开。
- 快速格式化 SD 卡后可恢复，说明问题更偏向文件系统写入/缓存/同步时序，而不是卡完全坏掉。

原因分析：

1. 采样过程中如果频繁打开/关闭文件，文件系统元数据会反复更新。
2. 如果写入期间掉电或拔卡，目录项和 FAT 可能不一致。
3. H743 开启 DCache 后，SDMMC DMA 读写如果没有做 cache 维护，容易出现 CPU 缓存与 DMA 实际内存不一致。
4. FatFs 的部分缓冲区可能不是 32 字节对齐，DMA 和 DCache 维护时需要 scratch buffer。

解决方式：

- `sd_diskio.c` 启用：

```c
#define ENABLE_SD_DMA_CACHE_MAINTENANCE  1
#define ENABLE_SCRATCH_BUFFER
```

- 对 SD DMA 读使用 `SCB_InvalidateDCache_by_Addr()`。
- 对 SD DMA 写使用 `SCB_CleanDCache_by_Addr()`。
- scratch buffer 改为 32 字节对齐：

```c
ALIGN_32BYTES(static uint8_t scratch[BLOCKSIZE]);
```

- 未对齐写入路径中，在 `memcpy()` 到 scratch 后、启动 SD DMA 写卡前，额外执行 DCache clean。
- `CsvLogger_AppendLine()` 保持文件打开状态时仍然每行 `f_sync()`，写失败会关闭文件。
- `VL53_App_SetMode(INFER)` 和采样停止路径都会调用 `CsvLogger_Close()`，降低文件长时间占用风险。

结果：

- SD DMA 与 DCache 的一致性问题已做维护。
- 未插卡不再应该让主程序直接卡死在 `Error_Handler()`。
- CSV 写入仍需注意：采样结束后应先发送 `STOP` 或等待 `DONE`，再断电或拔卡。

当前建议：

```text
采样结束后：
1. 等待 NORMAL DONE / BAL DONE / OK STOP。
2. 确认 LED2 灭。
3. 再断电或拔 SD 卡。
```

### 12.4 未插 SD 卡导致程序卡死问题

问题表现：

- 不插 SD 卡时，程序可能在 SD 初始化阶段进入错误路径。
- 这会导致串口、VL53、VOFA 都没有机会继续运行。

解决方式：

`MX_SDMMC1_SD_Init()` 中，`HAL_SD_Init(&hsd1)` 失败后不再 `Error_Handler()`，而是直接 `return`：

```c
if (HAL_SD_Init(&hsd1) != HAL_OK)
{
  return;
}
```

结果：

- 未插 SD 卡时主程序仍能继续跑。
- 真正需要写 CSV 时，由 FatFs / CsvLogger 返回错误码。
- 这让 H7 在没有 SD 卡时仍然可以作为推理和 CAN 主控使用。

### 12.5 VOFA 采样控制已经扩展为普通采样和均衡采样

当前 VOFA 命令已经明显多于早期版本。

采样相关命令：

| 命令 | 作用 |
|---|---|
| `MODE` | 采样模式 / 推理模式切换 |
| `SAMPLE` | 单次采样 |
| `REC` | 开始或停止连续采样 |
| `STOP` | 强制停止采样并切回推理模式 |
| `SETREC n ms` | 设置普通连续采样次数和间隔，同时切换为普通采样模式 |
| `BALCFG start end step target ms` | 设置距离均衡采样参数，同时切换为均衡采样模式 |
| `LABEL x` | 设置标签，`0~100` 或 `255` |
| `STATUS` | 查看当前模式、采样状态、标签、错误和均衡采样统计 |
| `ERRCLR` | 清除错误灯 |
| `HELP` | 输出命令说明 |

均衡采样的目的：

- 解决数据集中某些距离段样本过多、某些距离段样本过少的问题。
- 采样前先用当前 VL53 帧计算手部中心距离 `center_z_mm`。
- 按距离分桶，只保存目标距离档还没满的样本。
- 不符合距离范围或该桶已满时跳过，不写 CSV。

均衡采样输出格式：

```text
BALCNT label=50 total=xx/yy target=20
DIST   180  200  220 ...
COUNT    5    8   20 ...
```

结果：

- 可以直接观察每个距离档已经采了多少条。
- 适合补充边缘距离数据，例如 `180~220mm`、`380~420mm`。

### 12.6 采样超时和自动重试机制

问题表现：

- 连续采样过程中，LED2 曾经长亮。
- 串口看起来卡住。
- 可能是某次 SD 写入、VL53 状态或 FatFs 操作没有在预期时间返回。

解决方式：

VOFA 层加入采样超时控制：

```c
#define VOFA_SAMPLE_TIMEOUT_MS  3000U
#define VOFA_SAMPLE_MAX_RETRY   3U
```

行为：

- 每次 `Vofa_StartSample()` 后设置 `sample_deadline_ms`。
- 如果 `VL53_App_ConsumeSampleResult()` 超时未返回，VOFA 会结束本次等待。
- 连续采样中，如果未超过最大重试次数，会打印：

```text
WARN SAMPLE_TIMEOUT_RETRY x/3
```

- 超过重试次数后，强制停止采样，切回推理模式，打印：

```text
ERR SAMPLE_TIMEOUT
```

结果：

- LED2 长亮和采样状态卡死问题得到缓解。
- 发生异常时系统能回到可继续控制的状态，而不是一直等待。

### 12.7 VL53 有效距离、前景和 ROI 问题

当前 AI 前处理有效距离范围：

```c
#define VL53_APP_AI_VALID_MIN_MM        160
#define VL53_APP_AI_VALID_MAX_MM        450
#define VL53_APP_AI_FOREGROUND_MARGIN   120
```

有效 zone 判断：

```text
distance 在 160~450mm
target_status == 5 或 9
nb_target_detected > 0
```

前景判断：

```text
hand zone = 有效 zone 且 distance <= d_min + 120mm
```

遇到的问题：

- 只用距离判断前景时，手臂可能也会被算入前景。
- 远近变化会影响张握程度判断，容易让模型“偷学距离”。
- ROI 计算早期经常显示 `0~7` 整个区域，说明根据当前 FOV 和手掌尺寸估算出的 ROI 很容易覆盖全图。
- VL53L8CH 的视场角理解曾出现偏差，后续改为用更保守的等效半角计算。

当前处理方式：

- ROI 调试仍保留，用于观察手部区域估计，但当前 CNN 输入主要仍基于全图固定 `20x8x8` 特征。
- 不把动态 ROI 裁剪成可变输入，避免模型输入尺寸不稳定。
- 使用全图 mask 和全局复制特征，让模型自己学习“哪里是手、哪里可能是手臂干扰”。

当前结论：

```text
ROI 当前主要用于调试和手部位置观察，不是最终训练输入的唯一依据。
当前 CNN 输入保持固定 20x8x8，便于 CubeAI 部署。
```

### 12.8 CNN 模型和 CubeAI 部署问题

早期文档中的训练方案是 MLP：

```text
distance[64] + CNH[64][8] = 576
```

后续实际测试发现：

- MLP 在 300mm 附近表现较好。
- 离开主要训练距离后误差明显变大。
- 手臂干扰和距离变化会导致模型误判。

因此引入 CNN 版本：

```text
输入：20 x 8 x 8
输出：openness_score 0~100
```

20 个通道包含：

- 相对距离。
- 绝对距离归一化。
- 有效点 mask。
- 前景 mask。
- `target_status` 有效状态。
- `nb_target_detected`。
- 8 个 CNH bin。
- 复制到全图的手部中心 `x/y/z`。
- 前景 zone 数量。
- 前景外接矩形宽度和高度。

当前部署方式：

- PC 端训练脚本位于 `H743VIT6/Training`。
- CNN 模型导出 ONNX。
- 使用 STM32CubeAI 导入、Analyze、Validation Desktop 后生成代码。
- MCU 端通过 `GestureAI_Run()` 推理。
- `VL53_App_UpdateAiEstimate()` 得到 `openness_score`。

当前串口显示：

```text
当前手掌张握程度：xx
```

已经验证过：

```text
当前模型在主要采样距离内效果可以，实时显示比较直观。
```

仍需注意：

- 当前模型精度强依赖训练数据覆盖范围。
- `340~420mm` 或边缘距离误差曾变大，需要继续补充数据。
- 如果改 CSV 特征或前处理参数，必须重新训练并重新导出 CubeAI 模型。

### 12.9 X-CUBE-AI / DCache 相关问题

问题背景：

- 使用 CubeAI 后，DCache 不能轻易关闭，或者关闭选项不可用。
- H7 开启 DCache 后，SDMMC DMA、串口 DMA、AI 输入缓冲等都要注意缓存一致性。

当前处理：

- 主程序开启 ICache 和 DCache。
- SDMMC DMA 已在 `sd_diskio.c` 做 cache maintenance。
- VOFA UART RX buffer 使用 32 字节对齐。

原则：

```text
DMA 写内存：CPU 读之前 invalidate。
CPU 写内存给 DMA 读：DMA 启动前 clean。
```

当前仍需注意：

- 后续如果新增 I2C DMA、UART TX DMA 队列、SPI/QSPI DMA，都要检查对应 buffer 的对齐和 cache 维护。
- AI 输入当前由 CPU 写入并由 CPU 调用推理，通常不涉及外设 DMA，但仍要避免把 DMA buffer 直接作为 AI 输入使用。

### 12.10 W25Q64 保存 HOME / ACTUAL 位置记录

问题背景：

- W25Q64 的主要目的不是大量存训练数据，而是掉电后恢复触手位置参考。
- 需要保存 G4 反馈的实际多圈位置，以及用户设置的 HOME 初始点。

当前命令：

| 命令 | 作用 |
|---|---|
| `W25ID` | 读取 JEDEC ID |
| `W25TEST` | 擦写读回测试 |
| `W25SAVE` | 保存当前 G4 actual 位置 |
| `W25LOAD` | 读取最新 HOME 和 ACTUAL |
| `CANZERO` | 将当前 G4 actual 作为 H7 HOME，并保存 HOME / ACTUAL |
| `MSETZERO q0 q1 q2` | 手动设置 H7 HOME 并保存 |

遇到的问题：

- 早期记录扫描和结构体大小绑定较紧，存在写入后读取异常风险。
- 需要确保写入成功不是只看 HAL 返回，而要能断电后恢复。

解决方式：

- 使用固定 64 字节日志槽：

```c
#define W25Q64_APP_LOG_SLOT_SIZE    64UL
```

- 记录带 `magic/version/size/type/seq/position/tick/crc32`。
- 保存后立即读回，检查：

```text
magic/version/type/crc/memcmp
```

结果：

已经出现过成功输出：

```text
OK W25LOAD home_seq=33 home=136,-172,844 actual_seq=114 actual=136,-172,844
```

说明 W25Q64 保存、扫描、读回校验链路已经可用。

### 12.11 CANFD 与 G4 通讯问题

当前 H7 和 G4 使用 CANFD 进行控制。

H7 发送：

```text
标准 ID 0x120
CAN FD + BRS
32 bytes payload
q 单位 = rad * 100
```

G4 状态帧：

```text
标准 ID 0x180
32 bytes payload
actual_q[0..2] 位于 byte20~31
```

H7 当前命令：

| 命令 | 作用 |
|---|---|
| `CANSTAT` | 查看 G4 在线、错误、mask、pwm、actual_q |
| `CANSTOP` | 停止 G4 三路输出 |
| `CANZERO` | 当前 G4 actual 作为 H7 HOME 并保存 |
| `CANHOME` | 发送 HOME 绝对目标，带 RESET |
| `CANPOS/CANREL dq0 dq1 dq2` | 发送相对位置命令 |
| `CANABS q0 q1 q2` | 发送 `HOME + input` 的绝对目标 |

重要语义：

```text
CANZERO：
  H7 保存 HOME，不直接让 G4 修改编码器零点。

CANHOME：
  H7 向 G4 发送 HOME 绝对目标，让 G4 位置环回到初始点。

CANPOS/CANREL：
  相对 G4 最新 actual_q 加增量。

CANABS：
  相对 H7 HOME 的绝对目标。
```

曾遇到的问题：

- `CANPOS 1000 1000 1000` 含义容易误解。实际单位是 `0.01rad`，所以 `1000 = 10rad`。
- H7 显示 target 早期没有使用 `actual + input`，导致相对命令显示不直观。
- G4 端如果三电机不是同步动，需要看 `CANSTAT` 中的 `mask/pwm/flags/actual_q`。
- 若只有某个电机动，问题更可能在 G4 端 PWM/ADC/闭环执行链，不是 H7 只发了一路。

当前 H7 侧已做：

- 相对命令要求 G4 状态新鲜，否则返回 `CAN_STALE`。
- `CANHOME` 和 `CANABS` 发送绝对目标时带 RESET。
- `CANSTAT` 可以显示：

```text
ready, tx, g4_valid, online, age, rx, g4rx, g4tx, err,
last_seq, cmd, flags, mask, pending, pwm, aq0,aq1,aq2
```

### 12.12 触手映射控制层

H7 已经加入 `motor_control_app`，用于把 AI 输出转换成三电机目标。

当前输入：

```text
ai->x, ai->y              手部平面位置，-100~100
ai->bend_0_100            弯曲程度，100 - openness_score
ai->stiffness_0_100       根据 center_z 得到的刚度
ai->center_z_mm           手部距离
```

当前输出：

```text
pull_q[3]    相对 HOME 的三根线拉动量
target_q[3]  发给 G4 的三电机绝对目标
```

方向关系已经按实际机构修正：

```text
电机2收缩 -> 空间角度 -135°
电机3收缩 -> 空间角度 -15°
电机1收缩 -> 空间角度 105°
```

工作区定义：

```text
-45° 到 135° 为前方工作区
135° 到 -45° 为后方工作区
```

当前控制策略：

- `x/y` 只表达方向。
- `bend_0_100` 表达弯曲程度。
- `stiffness_0_100` 表达共同预紧/刚度。
- 不周期重复发送同一个目标，目标变化时发送一次，由 G4 位置环保持。

仍需注意：

- H7 只负责生成合理目标，G4 端位置环必须先稳定。
- 当前 G4 端仍在排查三电机位置环和方向一致性问题，因此 H7 的映射层需要等 G4 基础闭环稳定后再做最终联调。

### 12.13 当前还没完全解决的问题

1. G4 三电机位置环还在联调中。

H7 CAN 发送和 G4 状态解析已通，但如果 G4 端只有某一路动、KEY2 本地测试乱转，问题更可能在 G4 的编码器方向、位置环反馈符号、ADC/TIM 触发链或按键测试路径，不属于 H7 单侧能完全解决的问题。

2. 模型在边缘距离仍可能误差变大。

当前 CNN 在主要采样距离内效果可以，但边缘距离和手臂干扰仍需要继续补数据和验证。建议继续使用均衡采样补 `180~220mm`、`380~420mm` 等距离段。

3. ROI 仍主要用于调试。

ROI 目前不能完全解决手臂干扰，也不应直接改成动态尺寸输入。后续如果要强化 ROI，应先在 PC 端脚本验证再同步 MCU 前处理。

4. SD 卡仍需按流程停止后再拔卡。

即使已经做 DCache 维护和 `f_sync()`，FAT 文件系统仍不适合在写入中直接断电。大量采集后建议等待 `DONE/STOP`。

5. CubeMX 重新生成后需要复查用户区。

尤其要检查：

```text
main.c 中 Can_App_Init / MotorControl_Init / VL53_App_Init / Vofa_Init
main.c while 中 VL53_App_Task / Vofa_Task / MotorControl_Task
sdmmc.c 未插卡 return 逻辑
sd_diskio.c DCache 维护和 scratch buffer
Keil 工程是否包含新增模块
```

### 12.14 当前最新结论

H7 侧已经完成：

- VL53L8CH 8x8 距离读取。
- CNH 8 bin 输出。
- CSV 采集字段补齐到 709 列。
- VOFA 普通采样和距离均衡采样。
- SDMMC DCache 维护和未插卡不阻塞。
- CNN 训练脚本和 CubeAI 推理接入。
- 串口显示 `当前手掌张握程度：xx`。
- W25Q64 HOME / ACTUAL 位置记录保存和读回校验。
- CANFD 与 G4 状态帧解析和位置命令发送。
- 初步触手映射控制层。

当前下一阶段重点：

```text
先把 G4 三电机位置环、方向、CAN 执行路径彻底稳定；
再回到 H7，使用当前 AI 输出和映射层做 H7->G4 实机联调。
```

---

## 13. 2026-07-06 至 2026-07-18 后续问题与解决记录

本章从第 12 章停止的位置继续记录。每项按“现象、原因、处理、当前结果”整理；尚未完成实机闭环验证的内容明确标为“待验证”，避免把临时缓解误写成彻底解决。

### 13.1 2026-07-06：CNN 推理、映射控制和位置保存链路合并

现象：

- CNN 已能在 H7 上输出手掌张握程度，但模型输出、手部平面位置、三电机目标和 G4 实际角度之间缺少统一数据流。
- HOME、断电前 ACTUAL 和 G4 上电后的多圈机械角度基准容易混淆，错误改写多圈角度曾导致电机高速乱转。
- 相对位置命令显示的 TARGET 一度只是输入增量，不是“当前位置 + 增量”，不便判断位置环是否正确。

原因判断：

- AI 输出属于手势空间，G4 位置环使用机械多圈角度，两者之间必须经过 H7 映射层和统一的正负方向约定。
- HOME 是回零目标，ACTUAL 是保存时的实际位置，不能互相替代；恢复坐标时也不能直接改动 FOC 使用的原始机械角度链路。

处理方法：

- H7 建立 `x/y + bend + stiffness -> 三路相对拉索量 -> G4 位置目标` 的映射流程。
- W25Q64 保存每个触手的 HOME、ACTUAL 和控制配置，并通过 RESTORE 有效标志约束启动。
- 相对位置调试输出改为显示“当前实际位置 + 输入增量”得到的目标。
- 对涉及多圈角度基准的修改坚持先停止电机，再进行坐标同步，避免运行中直接替换闭环反馈量。

当前结果：

- CNN、VOFA、映射控制、CAN 和 W25Q64 已形成完整基础链路。
- 正常上电恢复可以使用保存记录继续工作，但“运行中断连后 G4 可能被外力移动”的情况不能使用 H7 旧 ACTUAL 覆盖 G4，见 13.9。

### 13.2 2026-07-07 至 2026-07-09：迁移到 H743VIT6 新板

现象：

- 新板按键、LED 引脚及有效电平与旧板不同，外部 Flash 由 QSPI 改为 SPI4。
- CubeMX/X-CUBE-AI 重新生成后出现 `ai_platform.h` 路径丢失、旧 QSPI 引用残留、时钟初始化进入 Error_Handler、未插 SD 卡进入错误流程等问题。
- VL53 在 PB8/PB9 上总线保持低电平，扫描不到设备；相同传感器在另一块主控板上正常。

原因判断：

- 新旧板不能只复制业务文件，CubeMX 外设配置、Keil include path 和生成后需要补回的用户代码必须同时迁移。
- AI 生成代码、SDMMC/FatFs 和 DCache 维护代码中存在不在稳定 USER CODE 区内的内容，重新生成后容易被覆盖。
- PB8/PB9 在新板上的实际硬件状态不满足 I2C 空闲高电平条件。

处理方法：

- 以 `H743VIT6(new)` 为新主工程，重新生成 X-CUBE-AI 并补回 AI 调用、未插卡不进入 Error_Handler、SD DCache 对齐缓冲区等逻辑。
- W25Q64 驱动改走 SPI4，清理业务层对旧 QSPI 句柄的依赖。
- VL53 I2C 从 PB8/PB9 迁移到 PB10/PB11；先用 100 kHz 排查，通信成功后恢复 400 kHz，并保留上电延时和初始化重试。

当前结果：

- 新板可完成 VL53 采集、CNN 推理、W25Q64 访问、CAN 和串口调试。
- CubeMX 每次重新生成后仍需复查 AI include path、SD 未插卡处理和 DCache 缓冲区，不能只看“生成成功”。

### 13.3 2026-07-09 至 2026-07-11：三拉索方向、放线限制和堵转保护

现象：

- POS 单独控制正常，但映射控制时可能出现某一根绳过松、另一根持续拉紧，电机发热或需要轻推后才能继续转动。
- 对三路目标分别限幅会破坏原有比例关系，导致方向改变、卷曲不足或反方向只能运动一部分。
- 堵转后保持原目标会持续输出电流，电机温升明显；清除错误后有时仍不能恢复。

原因判断：

- H7 显示方向、CAN 命令方向和实际收放线方向曾存在多层符号转换，板间差异进一步放大了混乱。
- 三拉索是关联运动，不能只截断某一路；单路限幅后，三路差分关系被破坏。
- 仅以位置变化判断堵转容易误判静摩擦，仅看电流又会把正常大负载当成堵转。

处理方法：

- 统一 H7 的语义：正值表示收线，负值表示放线；每块 G4 的实际差异只通过触手方向表修正。
- 当任一路达到放线边界时，对三路差分量进行同比例缩放，保持运动方向和三路相对关系。
- 当前默认参数为：弯曲增益 `27000 q`、刚度增益 `2000 q`、最大放线量 `30000 q`，其中 `1 q = 0.01 rad`。
- 堵转判断组合使用“大电流 + 目标误差较大 + 一段时间内实际位置变化很小”；触发后目标改为当前位置并退出持续保持，STOP 同时清理堵转状态。

当前结果：

- 同比例缩放避免了“只限制一根绳”造成的方向畸变，反方向运动范围也比固定小放线量更完整。
- 堵转逻辑已降低误触发，但减速机静摩擦、绳索预紧不一致和单电机异常发热仍需逐板实测，不能仅靠提高电流阈值解决。

### 13.4 2026-07-10 至 2026-07-12：从双触手扩展到四触手

现象：

- 第二块 G4 接入后，原先只保存一套 HOME/ACTUAL 和只使用一个 CAN 节点号的结构无法区分触手。
- 切换触手后可能读到上一触手的数据，设置零点和回零也容易操作错对象。
- 后续四块 G4 如果复制四份代码，维护和烧录时很容易出现参数版本不一致。

原因判断：

- CAN 地址、闭环反馈方向和 FOC 微调属于板级差异；HOME、ACTUAL 和映射配置属于触手运行数据，两类配置不能混为一套全局变量。

处理方法：

- G4 使用统一 `board_profiles.h`，通过板号宏选择 CAN 节点号、三电机闭环反馈方向和 FOC 微调参数。
- H7 将 4 个触手映射为固定索引和 CAN ID，TSEL 只选择调试对象，工作模式决定实际活动掩码。
- W25Q64 改为按触手保存运行记录，HOME 和 ACTUAL 合并到同一快照，只更新发生变化的触手。
- 保存区采用版本、序号、有效标志和 CRC 校验；擦除过程改为任务化处理，减少一次阻塞整个控制循环的时间。

当前结果：

- 单触手可独立选择，双触手支持 1/2 或 3/4 分组，四触手具备统一分发基础。
- 四块 G4 仍需分别烧录对应板号配置，但业务代码保持一份，避免四份源代码长期分叉。

### 13.5 2026-07-12 至 2026-07-13：工作模式和协同关系整理

现象：

- 单触手、双触手和四触手命令混用时，STOP、HOME、POS 和映射目标的作用对象不清楚。
- 两个触手安装方向对称，直接复制相同 `x/y` 会出现实际方向相差 180°。
- 四触手中心对称控制中，触手 4 曾出现三根线都放松，而期望是电机 1、3 收线，电机 2 放线。

处理方法：

- 保留 ABS 调试命令，但主要工作流统一为 `工作模式 + 活动触手掩码 + 关系类型 + 控制使能`。
- 工作模式划分为单触手、双触手 1/2、双触手 3/4 和四触手；关系包括同向、镜像和中心对称。
- STOP 和 HOME 对当前活动组生效；单触手 POS 保留独立调试，协同模式使用统一输入再按安装关系变换。
- 触手 3/4 按触手 1/2 整体翻转 180°后的安装关系处理，每个触手最终仍经过自己的三电机方向表。

当前结果：

- 工作对象和控制关系已从分散命令收敛到统一分发层。
- 触手 4 的方向表已修正，但四触手所有方向组合仍需逐个使用小幅度命令验证，不能直接以大 BEND 做首次测试。

### 13.6 2026-07-13：虚拟按键误触和串口调试链路

现象：

- 手势滤波、帧数和时间窗叠加后，虚拟按键既难触发又可能误触；进入按键模式后还会干扰正常映射控制。
- VOFA 串口从 USART1 切换到 USART7 后，曾出现只能接收 H7 输出、H7 收不到上位机命令的情况。
- USB 转串口关闭或重新打开后，UART7 RX DMA 可能停止；插拔接收器后又恢复。

原因判断：

- 虚拟按键状态机与映射控制、RESTORE 等状态耦合过多，当前比赛调试阶段收益低于风险。
- 环形 DMA 写指针在计数器瞬间为 0 时可能得到缓冲区长度，若不归一化会破坏首尾判断；UART 错误回调还需要确保重新启动 RX DMA。

处理方法：

- 当前先关闭 H7 的手势虚拟按键触发，保留代码和串口查询，避免无意切入按键模式。
- VOFA UART 使用编译开关选择 USART1/USART7，发送统一通过句柄函数获取，不再写死 `huart1`。
- RX DMA 写位置等于缓冲区长度时归一化为 0；串口异常恢复逻辑只重启接收，不错误处理发送队列。

当前结果：

- USART7 可用于当前上位机通信；一次“完全无命令响应”最终通过重新插拔 USB 转串口恢复，说明硬件/驱动状态也是风险来源。
- 比赛前应验证串口断开重连，不应只验证上电后的第一次连接。

### 13.7 2026-07-13 至 2026-07-14：PC/AI 控制源隔离和上位机状态同步

现象：

- 上位机处于 PC 控制时，AI 新帧仍可能更新目标；上位机显示的模式与 H7 实际执行模式有时不一致。
- 上位机需要知道当前来源、使能、活动触手、在线/就绪/故障掩码和映射输入，但不需要显示全部电机内部变量。
- G4 状态中的 `online` 原本表示“最近是否收到 H7 命令”，H7 又据此决定是否发送命令，形成互相等待。

处理方法：

- 新增 `MSRC AI/PC` 控制源仲裁。切换来源时先 STOP 并关闭映射；PC 模式屏蔽 AI 和虚拟按键目标，但不阻塞 RESTORE/W25 后台任务。
- 新增 `MCTRL 0/1` 作为统一运动许可；`MVIRT` 只在 PC 来源且控制已启用时接受。
- 新增 `SNAP?` 返回控制来源、使能、模式、活动掩码、在线/就绪/故障掩码和当前映射输入；新增 `POS?` 返回四个触手相对 HOME 的位置。
- H7 的在线判断改为“已收到有效状态帧且状态年龄未超限”，不再依赖 G4 原始 `online` 位。

当前结果：

- PC 和 AI 目标已隔离，上位机可以通过 SNAP/POS 同步主要状态。
- 上位机编辑了新模式但未下发时，界面草稿可能与 H7 实际模式不同；启动前还需做到“下发模式并由 SNAP 确认后再启用”，该项仍待上位机完善。

### 13.8 2026-07-14 至 2026-07-15：READY、运行健康和一次性停止原因

现象：

- READY 条件同时用于启动和运行，短暂状态帧延迟会立即拒绝 MVIRT、清除 enable 并 STOP。
- 上位机因此出现 `ERR MVIRT NOT_READY`，下一帧又变成 `ERR MVIRT DISABLED`，实际机构并未发生危险却被频繁停机。
- `last_tx_status=2` 曾同时表示未就绪和 CAN 队列忙，诊断含义不清楚。

原因判断：

- 启动前必须严格确认坐标可信，但运行中不能把一次短暂 CAN/READY 抖动等同于坐标失效。
- READY、FAULT、TX_BUSY 和 CAN_TIMEOUT 是不同问题，必须分开处理。

处理方法：

- 启动 `MCTRL 1` 时继续严格检查：活动 G4 在线、状态新鲜、无 FAULT、RESTORE 完成且位置恢复有效。
- 运行中改为健康监测：短暂状态波动继续接收 MVIRT；FAULT 立即停止；真正停机后不自动重新启动。
- 增加一次性停止原因：`CAN_TIMEOUT`、`FAULT`、`RESTORE_LOST`、`ESTOP` 和 `GROUP_SYNC`，同一事件不重复刷屏。
- 安全 STOP 按 100 ms 间隔最多补发 5 轮，避免无限占用 CAN FIFO。

当前结果：

- 启动安全和运行容错已分离，PC 不再因单帧 READY 波动立刻退出。
- 为排查后续运动时假超时，当前代码暂时关闭了“CAN 状态超时自动停机”，只保留告警；FAULT 和人工急停仍需立即处理。恢复自动停机前必须先完成 13.10 的通信压力测试。

### 13.9 2026-07-15：断连重连时的坐标恢复原则

现象：

- 如果 G4 与 H7 断连期间仍在运动或被外力转动，H7 保存的最后 ACTUAL 已经不是 G4 的真实位置。
- 重连后若继续把 H7 旧 ACTUAL 写回 G4，会把错误坐标强行恢复到 G4，后续 HOME 和位置目标全部偏移。

最终原则：

- 正常运行时：G4 ACTUAL -> H7 -> W25Q64。
- H7 上电且能确认 G4 未发生未知运动时，才允许使用已验证的启动恢复流程。
- 运行中断连后重连：先 STOP，使用 G4 当前 ACTUAL 更新 H7 和 W25Q64，禁止用 H7 旧 ACTUAL 覆盖 G4。
- 重连只恢复坐标和状态，不自动续跑旧目标；用户重新确认后才能再次 `MCTRL 1`。

当前结果：

- 该原则已确定，但“区分首次上电与运行中掉线”“G4 当前 ACTUAL 的可信确认”和“重连后的写入顺序”仍需完整实机验证。

### 13.10 2026-07-15 至 2026-07-17：运动时 CAN 超时和四触手不稳定

现象：

- 单触手、双触手和四触手静止时状态正常，一运动就可能出现 `MSTOP reason=CAN_TIMEOUT` 或 `GROUP_SYNC`。
- 停止后所有触手又恢复在线，说明并非持续断线。
- 单纯增加 H7 ACK 等待时间、提高中断优先级或降低上位机刷新频率没有从根本上解决。

原因定位：

- G4 状态上报曾依赖 `HAL_GetTick()`/SysTick；高频电机中断和运行负载变化时，软件时间基准及调度不稳定，状态帧间隔被拉长。
- 四触手放大了状态帧、目标帧和确认帧的并发压力，H7 的严格超时把“状态延迟”误判为“节点掉线”。

处理方法：

- G4 状态上报改用固定定时器节拍分频，不再依赖 HAL SysTick 判断发送时机。
- 状态上报周期暂定 50 ms；H7 继续以最新状态时间戳判断，不要求每个目标都同步阻塞等待。
- 组命令序号按回绕方式比较，确认窗口放宽到 800 ms；发送失败时保持上一组有效目标，不生成半组新目标。
- 调整 CAN 中断优先级，避免状态收发长期被低优先级任务拖延。

当前结果：

- 改为固定定时器节拍后，实测“基本不卡”，这是本轮 CAN 稳定性改善最明显的修改。
- 50 ms 已能满足当前状态监测，没有必要立即改成 20 ms；四块 G4 全部烧录相同修复后再统一压力测试。

### 13.11 2026-07-17：W25Q64 保存失败与参数边界

现象：

- 重新烧录 H7 后有时需要重新 CANZERO 才显示 READY，表现得像 HOME/ACTUAL 丢失。
- 设置 `MCFG 32000 2500 ...` 或 `MCFG 30000 2500 ...` 时重复返回 `ERR MCFG_SAVE status=1`。

原因判断：

- 烧录内部 Flash 本身不会清除外部 W25Q64；问题更可能来自记录版本、有效标志、CRC、当前触手索引或启动时错误覆盖。
- 当前参数合法范围限制弯曲增益最大 `30000 q`、刚度增益最大 `2000 q`，`2500` 超出范围，因此保存被拒绝，不是 W25 容量溢出。

处理方法：

- HOME 和 ACTUAL 使用同一触手快照，保存时只覆盖确认更新的触手；旧有效记录在新记录校验成功前保留。
- 擦除和写入由任务状态机分步执行，避免一次长阻塞影响推理和 CAN。
- 参数写入前先做范围检查；需要更大刚度时，应先评估电机温升和绳索预紧，再同时修改范围和控制限幅，不能只放开 VOFA 检查。

当前结果：

- 当前可用默认值为 `bend=27000 q`、`stiff=2000 q`、`release_limit=30000 q`。
- W25 快照和非阻塞擦除已进入代码，但仍需做多次断电、烧录和记录区写满后的循环压力测试。

### 13.12 2026-07-18：当前尚未闭环的问题

#### 13.12.1 随机单触手持续弯曲且 STOP 无效

现象：

- 四触手运行中偶尔只有一个触手继续弯曲，其他触手不再响应；上位机姿态和 ACTUAL 仍持续变化，但弯曲输入值没有继续增大。
- 有时 CANSTOP 也不能立即让该触手停下，断电重连后 ACTUAL 又恢复正常。

当前判断：

- 不能只归因于上位机，因为 H7 显示输入不变而 G4 实际位置仍在变化。
- G4 的 STOP 当前经过 CAN 接收缓存和主循环任务后才真正关闭 PWM，若 STOP pending 被后续位置命令覆盖，或主循环被高频任务拖延，可能出现“状态帧仍在线，但命令没有及时执行”。
- 还需检查编码器 DMA 新鲜度；`valid=1` 不能代替“最近确实收到新角度”。

后续解决方案：

1. G4 增加独立的高优先级 STOP 锁存，STOP 不能被后续位置命令覆盖。
2. G4 增加编码器样本年龄看门狗，角度长期不更新时禁止继续位置环输出。
3. H7 不只确认 STOP 已入 CAN FIFO，还要等待 G4 状态确认 PWM 已关闭。
4. 增加“目标误差持续扩大”保护，防止方向错误时继续拉紧。

状态：待实现和实机验证，比赛前必须优先解决。

#### 13.12.2 上位机显示模式与 H7 实际模式不一致

现象：

- 在未启动时切换模式或锁定参数，上位机可能显示新的协同/四触手模式，但启动后 H7 返回的仍是 SOLO。

原因判断：

- 上位机界面保存的是未下发的模式草稿，周期 SNAP 又保留草稿显示；启动检查没有要求草稿先写入 H7 并得到确认。

后续解决方案：

- 启动按钮执行固定顺序：发送工作模式 -> 等待 SNAP 确认 -> 发送 `MCTRL 1`。
- 模式未确认时禁止发送 MVIRT；运行中点击三维触手只改变界面选择，不发送会停止活动组的 TSEL。

状态：上位机待修改，H7 的 SNAP 格式无需扩展。

### 13.13 截至 2026-07-18 的结论和验证顺序

已经确认可用：

- 新 H743VIT6 板上的 VL53、CNN、SPI4 W25Q64、USART7 和 CAN 基础链路。
- 单触手 POS、AI/PC 来源隔离、SNAP/POS 查询和多触手工作分发。
- G4 使用固定定时器节拍每 50 ms 上报状态后，运动时 CAN 卡顿明显改善。

当前优先级：

1. 先完成 G4 STOP 锁存和编码器新鲜度保护，复现并消除“单触手持续弯曲且无法停止”。
2. 四块 G4 全部使用同一版 50 ms 定时状态上报代码，依次做单触手、双触手、四触手压力测试。
3. 修正上位机模式草稿与 H7 实际模式的启动确认顺序。
4. 验证四触手方向表，特别是触手 4 的电机 1/2/3 收放线关系。
5. 做 W25Q64 多次断电、重新烧录和写满回绕测试。
6. 上述安全项通过后，再提高 BEND/STIFF 和抓取力度，不能用增益补偿尚未解决的方向或停止问题。
