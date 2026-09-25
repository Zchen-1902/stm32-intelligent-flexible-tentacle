# ACT USART1 混合协议 V1

## 1. 当前协议基线

本文件与 `H743VIT6_ACT/Core/act_stream/act_stream.c` 第一阶段实现一致：

```text
PC -> H7：ASCII命令，以CRLF结束
H7 -> PC：带长度和CRC32的二进制包
```

当前阶段只用于串口通信、H7同步和触手选择验证，不控制电机、不发送ACT_FRAME。

## 2. 串口参数

第一阶段：

```text
115200 bit/s
8 data bits
1 stop bit
no parity
no flow control
```

接入20 Hz完整ACT_FRAME前，PC和H7同时改为2,000,000 bit/s。

## 3. PC文本命令

每条命令以`\n`或`\r\n`结束。PC统一发送CRLF。

```text
ACT PING
ACT SELECT 1
ACT SELECT 2
ACT SELECT 3
ACT SELECT 4
ACT STATUS
```

命令编号：

```text
0 UNKNOWN
1 PING
2 SELECT
3 STATUS
```

H7当前命令行缓冲区为48字节。PC不得在一条命令中注入额外换行。

## 4. H7二进制回复

多字节字段均为little-endian：

| 偏移 | 类型 | 字段 | 说明 |
|---:|---|---|---|
| 0 | `uint16` | `magic` | 固定`0xA55A`，线上字节`5A A5` |
| 2 | `uint8` | `type` | 包类型 |
| 3 | `uint8` | `version` | 当前为1 |
| 4 | `uint16` | `data_length` | data字节数 |
| 6 | `uint8[N]` | `data` | payload |
| 6+N | `uint32` | `crc32` | 对头部和data计算，不含CRC本身 |

固定开销为10字节。当前包类型：

```text
1 ACK
2 STATUS
3 ERROR
```

解析器必须支持半包、粘包、无效前缀、CRC错误和重新寻找magic。PC最大允许data长度暂定8192字节，为后续约4.2KB的ACT_FRAME留出余量。

## 5. CRC32

CRC与Python `zlib.crc32()`一致：

```text
反射多项式：0xEDB88320
初值：0xFFFFFFFF
最终异或：0xFFFFFFFF
check("123456789") = 0xCBF43926
```

## 6. ACK和ERROR

ACK与ERROR的data均为2字节：

```text
data[0] command_id
data[1] result
```

结果编号：

```text
0 OK
1 BAD_COMMAND
2 BAD_ARGUMENT
```

`ACT STATUS`直接返回STATUS，不再返回ACK。

## 7. STATUS

当前data固定12字节：

| 类型 | 字段 |
|---|---|
| `uint8` | selected_tentacle |
| `uint8` | control_enabled |
| `uint8` | capture_state |
| `uint8` | reserved |
| `uint32` | sent_frame_count |
| `uint32` | dropped_frame_count |

后续可以在12字节之后追加ONLINE、READY和FAULT字段，但不能改变现有前12字节顺序。PC必须按data_length兼容解析。

## 8. 请求—应答规则

第一阶段没有通用命令sequence，因此PC同时最多只能等待一个状态请求：

```text
ACT PING -> PING ACK
ACT STATUS -> STATUS
ACT SELECT n -> SELECT ACK -> ACT STATUS -> STATUS
```

PC打开串口后，只有PING ACK和STATUS都合法才显示“H7已同步”。打开COM不等于协议同步。

PING超时允许重发一次。SELECT不自动重发，防止未来协议扩展后重复执行有副作用的命令。

## 9. 后续命令预留

第二阶段计划：

```text
ACT ENABLE 1
ACT ENABLE 0
ACT MOVE x y bend stiff
ACT HOME
ACT STOP
ACT REC 1
ACT REC 0
```

这些命令在H7实现前，PC界面必须保持对应控件禁用。

`ACT MOVE`最高20 Hz，只发送最新目标，不等待逐帧ACK。STOP保持最高优先级。

## 10. 后续数据包待冻结项

第二阶段需要定义实时姿态TELEMETRY，至少包含选中触手的状态和`actual_q[3]`。

第三阶段需要定义ACT_FRAME精确字段偏移。新需求采用10个CNH区间和未来12帧actual监督标签，但当前H7手势通道仍为8个CNH区间，必须先完成独立ACT采集模式，不能直接修改现有手势CNN通道宏。

