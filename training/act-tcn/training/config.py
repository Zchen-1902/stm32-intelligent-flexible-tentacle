"""Tiny ToF-ACT数据接口的固定参数。"""

SAMPLE_RATE_HZ = 16
GRID_SIZE = 8
ZONE_COUNT = GRID_SIZE * GRID_SIZE
TOF_CHANNELS = 18
CNH_BINS = 10
MOTOR_COUNT = 3

# 每个训练样本使用最近12帧观测，并监督之后10帧动作。
HISTORY_FRAMES = 12
ACTION_CHUNK_FRAMES = 10

# H7中定义的ACT电机状态有效位。
POSITION_VALID_FLAG = 0x01
ACTION_VALID_FLAG = 0x02

# 与H7现有VL53有效区判断保持一致。
VALID_TARGET_STATUS = (5, 9)

# 一条原始episode必须包含的数组字段。
REQUIRED_EPISODE_FIELDS = (
    "frame_seq",
    "timestamp_ms",
    "distance_mm",
    "target_status",
    "target_count",
    "signal_per_spad",
    "ambient_per_spad",
    "reflectance",
    "range_sigma_mm",
    "cnh_raw",
    "cnh_scaler",
    "valid_flags",
    "actual_q",
    "action_q",
)

# Tiny ToF-ACT第一版模型尺寸，固定后便于ONNX和Cube.AI分析。
MODEL_DIM = 64
ATTENTION_HEADS = 4
ENCODER_LAYERS = 2
DECODER_LAYERS = 1
FEEDFORWARD_DIM = 128
LATENT_DIM = 16
MODEL_DROPOUT = 0.10

# 归一化标准差低于该值时按1处理，避免常量通道除零。
NORMALIZATION_EPS = 1.0e-6

# 18个ToF通道的固定顺序必须与preprocess.py和H7保持一致。
TOF_CHANNEL_NAMES = (
    "distance_mm",
    "valid_mask",
    "target_status",
    "target_count",
    "signal_per_spad",
    "ambient_per_spad",
    "reflectance",
    "range_sigma_mm",
) + tuple(f"cnh_bin_{index}" for index in range(CNH_BINS))
