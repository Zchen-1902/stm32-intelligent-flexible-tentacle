"""可由STM32 Cube.AI稳定部署的Tiny ToF-TCN模型。"""

from __future__ import annotations

import copy

import torch
from torch import nn

from training.config import (
    ACTION_CHUNK_FRAMES,
    GRID_SIZE,
    HISTORY_FRAMES,
    MODEL_DIM,
    MOTOR_COUNT,
    TOF_CHANNELS,
)
from training.model import TinyTofAct, TofFrameEncoder
from training.normalization import NormalizationStats


class TinyTofTcn(nn.Module):
    """根据12帧ToF和电机位置预测未来10帧三电机位置。

    时序部分只使用两层固定长度卷积，不包含注意力、LayerNorm或动态形状
    运算，便于STM32H743上的Cube.AI生成和数值一致性验证。
    """

    def __init__(self) -> None:
        super().__init__()
        self.tof_encoder = TofFrameEncoder()
        self.state_encoder = nn.Sequential(
            nn.Linear(MOTOR_COUNT, 16),
            nn.ReLU(inplace=False),
            nn.Linear(16, 16),
            nn.ReLU(inplace=False),
        )
        self.temporal_encoder = nn.Sequential(
            nn.Conv2d(MODEL_DIM + 16, MODEL_DIM, kernel_size=(3, 1), padding=(1, 0)),
            nn.ReLU(inplace=False),
            nn.Conv2d(MODEL_DIM, MODEL_DIM, kernel_size=(3, 1), padding=(1, 0)),
            nn.ReLU(inplace=False),
        )
        self.action_head = nn.Sequential(
            nn.Linear(HISTORY_FRAMES * MODEL_DIM, 128),
            nn.ReLU(inplace=False),
            nn.Linear(128, ACTION_CHUNK_FRAMES * MOTOR_COUNT),
        )

    def initialize_from_teacher(self, teacher: TinyTofAct) -> None:
        """复制教师模型中可复用的单帧视觉编码器和电机状态编码器。

        Args:
            teacher: 已加载训练权重的Transformer教师模型。

        Note:
            只复制结构完全相同的两部分；TCN时序卷积和动作头仍按新模型
            初始化，避免错误地映射不兼容的Transformer权重。
        """

        self.tof_encoder.load_state_dict(teacher.tof_encoder.state_dict())
        self.state_encoder.load_state_dict(teacher.state_encoder.state_dict())

    @staticmethod
    def _check_inputs(
        tof_history: torch.Tensor,
        state_history: torch.Tensor,
    ) -> None:
        """检查训练接口的固定输入形状，提前暴露数据接线错误。"""

        if tof_history.ndim != 5 or tuple(tof_history.shape[1:]) != (
            HISTORY_FRAMES,
            TOF_CHANNELS,
            GRID_SIZE,
            GRID_SIZE,
        ):
            raise ValueError("tof_history尺寸必须为[B,12,18,8,8]")
        if state_history.ndim != 3 or tuple(state_history.shape[1:]) != (
            HISTORY_FRAMES,
            MOTOR_COUNT,
        ):
            raise ValueError("state_history尺寸必须为[B,12,3]")
        if tof_history.shape[0] != state_history.shape[0]:
            raise ValueError("ToF和电机历史的batch大小必须一致")

    def forward(
        self,
        tof_history: torch.Tensor,
        state_history: torch.Tensor,
    ) -> torch.Tensor:
        """执行TCN推理。

        Args:
            tof_history: 已归一化的``[B,12,18,8,8]`` ToF历史。
            state_history: 已归一化的``[B,12,3]``电机实际位置历史。

        Returns:
            归一化的``[B,10,3]``未来电机位置。
        """

        self._check_inputs(tof_history, state_history)
        batch_size = tof_history.shape[0]
        frames = tof_history.reshape(
            batch_size * HISTORY_FRAMES,
            TOF_CHANNELS,
            GRID_SIZE,
            GRID_SIZE,
        )
        tof_tokens = self.tof_encoder(frames).reshape(
            batch_size,
            HISTORY_FRAMES,
            MODEL_DIM,
        )
        state_tokens = self.state_encoder(state_history)
        tokens = torch.cat((tof_tokens, state_tokens), dim=-1)
        temporal = self.temporal_encoder(tokens.transpose(1, 2).unsqueeze(-1))
        action = self.action_head(temporal.flatten(1))
        return action.reshape(batch_size, ACTION_CHUNK_FRAMES, MOTOR_COUNT)


class TcnDeploymentModel(nn.Module):
    """固定batch=1并内嵌归一化的STM32部署模型。"""

    def __init__(self, source: TinyTofTcn, stats: NormalizationStats) -> None:
        """复制训练权重并注册归一化常量。

        Args:
            source: 已加载checkpoint且处于eval模式的TCN模型。
            stats: 与checkpoint配套的训练集归一化统计量。

        Note:
            部署输入ToF为``[1,13824]``连续数组。逐帧固定展开CNN可避免
            Cube.AI对动态batch/reshape推导产生不一致结果。
        """

        super().__init__()
        self.tof_encoder = copy.deepcopy(source.tof_encoder)
        self.state_encoder = copy.deepcopy(source.state_encoder)
        self.temporal_encoder = copy.deepcopy(source.temporal_encoder)
        self.action_head = copy.deepcopy(source.action_head)
        self.register_buffer(
            "tof_mean",
            torch.as_tensor(stats.tof_mean).reshape(1, TOF_CHANNELS, 1, 1),
        )
        self.register_buffer(
            "tof_std",
            torch.as_tensor(stats.tof_std).reshape(1, TOF_CHANNELS, 1, 1),
        )
        self.register_buffer(
            "state_mean",
            torch.as_tensor(stats.state_mean).reshape(1, 1, MOTOR_COUNT),
        )
        self.register_buffer(
            "state_std",
            torch.as_tensor(stats.state_std).reshape(1, 1, MOTOR_COUNT),
        )
        self.register_buffer(
            "action_mean",
            torch.as_tensor(stats.action_mean).reshape(1, 1, MOTOR_COUNT),
        )
        self.register_buffer(
            "action_std",
            torch.as_tensor(stats.action_std).reshape(1, 1, MOTOR_COUNT),
        )

    def forward(
        self,
        tof_history: torch.Tensor,
        state_history: torch.Tensor,
    ) -> torch.Tensor:
        """由原始12帧观测输出未来10帧三电机目标q。

        Args:
            tof_history: ``[1,13824]``，12帧连续ToF特征。
            state_history: ``[1,12,3]``，电机实际位置，单位0.01rad。

        Returns:
            ``[1,10,3]``目标位置，单位0.01rad。
        """

        frame_values = TOF_CHANNELS * GRID_SIZE * GRID_SIZE
        tof_tokens = []
        for index in range(HISTORY_FRAMES):
            begin = index * frame_values
            frame = tof_history[:, begin : begin + frame_values].reshape(
                1,
                TOF_CHANNELS,
                GRID_SIZE,
                GRID_SIZE,
            )
            frame = (frame - self.tof_mean) / self.tof_std
            tof_tokens.append(self.tof_encoder(frame).reshape(1, 1, MODEL_DIM))

        normalized_state = (state_history - self.state_mean) / self.state_std
        state_tokens = self.state_encoder(normalized_state)
        tokens = torch.cat((torch.cat(tof_tokens, dim=1), state_tokens), dim=-1)
        temporal = self.temporal_encoder(tokens.transpose(1, 2).unsqueeze(-1))
        normalized_action = self.action_head(temporal.flatten(1)).reshape(
            1,
            ACTION_CHUNK_FRAMES,
            MOTOR_COUNT,
        )
        return normalized_action * self.action_std + self.action_mean
