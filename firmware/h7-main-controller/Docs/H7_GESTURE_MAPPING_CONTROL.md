# H7 手势到三线触手映射控制逻辑

## 1. 文档目的

本文档用于记录 H7 主控侧的手势映射控制逻辑，方便后续把 VL53 传感器数据、AI 推理结果和三电机位置控制连接起来。

当前控制链路为：

```text
VL53 8x8 距离/CNH 数据
  -> H7 手部位置与张握程度判断
  -> 归一化虚拟控制量 x / y / bend / stiffness
  -> 三根绳子的拉紧量 pull0 / pull1 / pull2
  -> 三台电机绝对位置 target0 / target1 / target2
  -> CAN FD 一帧同步发送给 G4
  -> G4 三电机位置伺服执行
```

G4 只负责三电机闭环执行，不理解触手模型。触手映射、手势解释、轨迹规划和 CAN 目标生成都放在 H7。

## 2. 输入变量定义

H7 映射层使用四个虚拟输入：

```text
x           手在传感器平面内的 X 方向，范围 -100~100。
y           手在传感器平面内的 Y 方向，范围 -100~100。
bend        弯曲程度，范围 0~100。
stiffness   刚度/预紧程度，范围 0~100。
```

含义如下：

```text
x/y:
  只决定触手弯曲方向，不直接决定弯曲幅度。

bend:
  决定触手卷曲/弯曲程度。

stiffness:
  决定三根绳子的共同拉紧量，用于改变整体预紧和刚度。
```

注意：手在平面内离中心越远，并不表示触手弯得越厉害。平面位置只用于生成方向，弯曲幅度由张握程度控制。

## 3. VL53 和 AI 输出的使用方式

VL53 提供手部空间位置和距离信息：

```text
hand_x / hand_y:
  根据 8x8 距离阵列中手部区域的中心位置计算。

hand_z:
  根据手部区域的平均距离或加权距离计算。
```

AI 推理输出手部张开程度：

```text
openness_score:
  0   = 接近握拳
  100 = 接近张开
```

如果控制上使用“握紧程度”作为弯曲量，则需要转换：

```text
grip = 100 - openness_score
bend = grip
```

如果后续模型直接输出“握紧程度”，则可以直接使用：

```text
bend = grip_score
```

距离 `hand_z` 用于刚度控制：

```text
hand_z 近  -> stiffness 增大，三根绳子共同拉紧，触手更硬。
hand_z 远  -> stiffness 减小，三根绳子共同放松，触手更软。
```

## 4. 方向映射规则

手部平面位置先转换为归一化方向输入：

```text
x = normalize(hand_x)
y = normalize(hand_y)
```

为了避免手在中心附近轻微抖动导致方向突变，需要使用中心死区：

```text
if sqrt(x*x + y*y) < direction_deadband:
    不更新弯曲方向
```

中心死区内推荐保持上一帧有效方向，而不是把方向强行置零。这样可以避免触手目标方向在中心附近来回跳变。

方向半径只用于判断方向是否可靠：

```text
半径很小：方向无效或保持上一方向。
半径中等：方向逐步更新。
半径较大：方向稳定采用当前手部方向。
```

方向半径不参与 `bend` 的计算，避免手在平面内轻微移动就改变弯曲幅度。

## 5. 三根绳子的空间方向

三根绳子按 120 度分布，当前使用整数近似方向向量：

```text
cable0 = (100,   0)
cable1 = (-50,  87)
cable2 = (-50, -87)
```

每根绳子在当前手部方向上的投影为：

```text
projection_i = (x * cable_x[i] + y * cable_y[i]) / 100
projection_i = clamp(projection_i, -100, 100)
```

投影含义：

```text
projection_i > 0:
  当前弯曲方向更接近第 i 根绳子的方向，该绳子需要更多拉紧。

projection_i < 0:
  当前弯曲方向远离第 i 根绳子的方向，该绳子需要相对放松。
```

三根 120 度方向向量近似满足：

```text
projection0 + projection1 + projection2 ~= 0
```

因此差动弯曲不会明显改变三根绳子的平均拉紧量，平均拉紧量由 `stiffness` 单独控制。

## 6. 拉绳量计算

H7 先在“拉绳空间”计算目标。

共同拉紧量：

```text
common_pull_q = stiff_gain_q * stiffness / 100
```

方向弯曲拉紧量：

```text
bend_pull_q[i] = bend_gain_q * bend * projection_i / 10000
```

每根绳子的最终拉紧量：

```text
pull_q[i] = common_pull_q + bend_pull_q[i]
```

这里的 `pull_q` 单位为 `0.01rad`，表示等效到电机侧的位置变化量。

约定：

```text
pull_q > 0 表示收绳/拉紧。
pull_q < 0 表示放绳/放松。
```

## 7. 拉绳量到电机目标

当前机械方向约定：

```text
电机正转 = 放长绳子
电机反转 = 缩短绳子
```

因此拉绳量到电机目标需要取反：

```text
target_q[i] = neutral_q[i] - pull_q[i]
```

其中：

```text
neutral_q[i]:
  第 i 台电机的中立位置，单位 0.01rad。
  通常对应触手伸直、绳子刚好拉紧的参考位置。

target_q[i]:
  发送给 G4 的第 i 台电机绝对位置目标，单位 0.01rad。
```

最终目标需要限幅：

```text
target_q[i] = clamp(target_q[i], target_min_q, target_max_q)
```

## 8. CAN FD 下发

H7 每次生成新的三电机位置目标后，通过 CAN FD 一帧同步发送给 G4。

位置目标使用 `int32` 小端格式：

```text
target_q = target_rad * 100
```

一帧同时包含三台电机目标：

```text
motor0_target_q
motor1_target_q
motor2_target_q
```

三台电机必须作为一个同步动作组下发，不建议单独覆盖其中一路目标。

## 9. 当前代码对应关系

当前 H7 代码中，虚拟输入接口为：

```c
MotorControl_SetVirtualInput(x, y, bend, stiffness);
```

对应文件：

```text
H743VIT6/Core/motor_control_app/motor_control_app.c
H743VIT6/Core/motor_control_app/motor_control_app.h
```

当前核心映射关系为：

```text
input_x/input_y
  -> projection_i
  -> bend_pull_q[i]

stiffness
  -> common_pull_q

common_pull_q + bend_pull_q[i]
  -> pull_q[i]

neutral_q[i] - pull_q[i]
  -> target_q[i]
```

## 10. 推理控制触发时机

AI 推理不应该放在中断中，也不应该在主循环中无条件反复运行。

推荐触发条件：

```text
VL53_App_Task() 读取到一帧新的有效数据
  -> 标记 new_frame
  -> 主循环中的手势控制任务消费 new_frame
  -> 构造 AI 输入
  -> 调用 GestureAI_Run()
  -> 更新 x/y/bend/stiffness
  -> 必要时发送新的三电机目标
```

第一版建议推理频率跟随 VL53 新帧频率，例如 10Hz。G4 侧位置环负责平滑执行，不需要 H7 在没有新输入时重复发送同一个目标。

## 11. 调试观察量

H7 侧建议观察：

```text
x
y
bend
stiffness
pull_q[0..2]
target_q[0..2]
CAN 发送次数
CAN 最后状态
```

G4 侧建议观察：

```text
三电机实际多圈位置
三电机目标位置
三电机速度
三电机 iq/id
CAN 命令接收计数
CAN 状态帧
```

## 12. 注意事项

1. 平面位置只控制方向，不控制弯曲幅度。
2. 张握程度控制弯曲幅度。
3. 距离控制刚度/预紧。
4. 三根绳子的差动量应尽量保持均值为 0。
5. 三根绳子的共同量只由 `stiffness` 控制。
6. 电机正转是放绳，收绳目标需要让电机位置减小。
7. 三台电机目标必须一帧同步下发，避免触手形态被单路目标覆盖破坏。
8. AI 推理只在 VL53 新帧到来后运行，不能放进高频中断。
9. 中心死区内不要让方向乱跳，应保持上一有效方向或缓慢回中。
10. 后续如果加入轨迹规划，应在 H7 侧对 `x/y/bend/stiffness` 或 `target_q[3]` 做斜坡，而不是让 G4 理解触手模型。
