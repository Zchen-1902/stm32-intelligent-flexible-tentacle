# H743 VL53L8CH 张握程度训练说明

本目录用于电脑端训练手掌张握程度模型。目标是把采集到的 `gesture.csv` 转换成一个可交给 STM32Cube.AI 的 ONNX 模型。

## 训练目标

输入一帧 VL53L8CH 数据：

```text
distance[64] + CNH[64][8] = 576 维
```

输出一个连续数值：

```text
openness_score = 0~100
0   表示完全握拳
100 表示完全张开
```

注意：这不是分类模型。即使采集标签是 `0/25/50/75/100`，模型输出仍然可以是 `37、62、88` 这样的连续值。

## CSV 字段要求

当前 MCU 采集的 CSV 应包含 709 列：

```text
timestamp_ms,label_score,frame_count,mode,valid,
d0...d63,
target_status_z0...target_status_z63,
nb_target_z0...nb_target_z63,
cnh_z0_b0...cnh_z63_b7
```

训练脚本第一版只把下面 576 维作为模型输入：

```text
d0...d63
cnh_z0_b0...cnh_z63_b7
```

下面两组字段暂时只用于清洗数据，不直接喂给模型：

```text
target_status_z0...target_status_z63
nb_target_z0...nb_target_z63
```

## 标签规则

参与回归训练的标签：

```text
0~100
```

不参与回归训练的标签：

```text
255 = 无手/无效
```

无手数据后续可以单独训练“有手/无手”判断。第一版先把张握程度回归模型训稳定。

## 依赖

电脑端需要 Python 环境，并安装：

```bash
pip install numpy torch
```

如果 `torch.onnx.export` 报 ONNX 相关错误，再安装：

```bash
pip install onnx
```

## 训练命令

在工程根目录 `D:/STM32Cube_project/4.8` 下运行：

```bash
python H743VIT6/Training/train_openness.py --csv G:/gesture.csv --epochs 300
```

也可以指定输出目录：

```bash
python H743VIT6/Training/train_openness.py --csv G:/gesture.csv --out H743VIT6/Training/exported --epochs 300
```

## 模型大小选择

默认模型：

```text
576 -> 32 -> 16 -> 1
```

如果验证误差偏大，可以试更大的隐藏层：

```bash
python H743VIT6/Training/train_openness.py --csv G:/gesture.csv --hidden 64 32 --epochs 300
```

## CNN 训练版本

如果 MLP 在不同距离或手臂干扰下误差偏大，可以使用 CNN 版本。CNN 输入为固定特征图：

```text
20 x 8 x 8
```

其中包含相对距离、有效点mask、前景mask、target_status、nb_target、8个CNH bin，以及复制到全图的中心位置/距离/前景框统计特征。CNN 前景参数和当前 MCU 侧保持一致：

```text
有效距离：160~450mm
前景区域：d_min + 120mm
```

训练命令：

```bash
python H743VIT6/Training/train_openness_cnn.py --csv H743VIT6/Training/data/gesture_z220_400_plus50_clean_abs40.csv --out H743VIT6/Training/exported_cnn --epochs 300
```

输出文件：

```text
gesture_openness_cnn.onnx   给 STM32Cube.AI 转换使用
gesture_openness_cnn.pt     PyTorch checkpoint
cnn_norm_params.json        20个通道的归一化参数
cnn_train_report.json       训练报告
```

## 数据清洗逻辑

脚本默认保留：

```text
valid == 1
label_score 在 0~100
有效 zone 数量 >= 3
```

有效 zone 的判断：

```text
target_status == 5 或 9
nb_target_detected > 0
```

如果只是想先验证训练脚本能跑，可以临时关闭质量过滤：

```bash
python H743VIT6/Training/train_openness.py --csv G:/gesture.csv --no-quality-filter
```

## 输出文件

训练完成后输出到 `H743VIT6/Training/exported/`：

```text
gesture_openness.onnx   给 STM32Cube.AI 转换使用
gesture_openness.pt     PyTorch checkpoint，方便复现
norm_params.json        输入归一化参数，MCU 推理前必须使用同一套参数
train_report.json       训练样本数、标签分布、误差指标
```

## 评价指标

脚本会打印：

```text
MAE
RMSE
最大误差
```

这里的误差单位就是 `0~100` 分数。

第一版建议目标：

```text
MAE <= 8~10：可用
MAE <= 5：较好
```

如果 MAE 很大，优先检查数据质量，而不是先加大模型。

## 后续接入 CubeAI

部署时需要保持同样的数据顺序：

```text
d0...d63
cnh_z0_b0...cnh_z63_b7
```

并使用 `norm_params.json` 中的参数做同样归一化：

```text
x_norm = (x - mean) / std
```

CubeAI 推理输出是 `0~1`，最终再转换为：

```text
openness_score = output * 100
```
