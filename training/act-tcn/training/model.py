"""面向STM32H7部署的Tiny ToF-ACT模型。"""

from __future__ import annotations

import torch
from torch import nn

from training.config import (
    ACTION_CHUNK_FRAMES,
    ATTENTION_HEADS,
    DECODER_LAYERS,
    ENCODER_LAYERS,
    FEEDFORWARD_DIM,
    GRID_SIZE,
    HISTORY_FRAMES,
    LATENT_DIM,
    MODEL_DIM,
    MODEL_DROPOUT,
    MOTOR_COUNT,
    TOF_CHANNELS,
)


class TofFrameEncoder(nn.Module):
    """使用同一组CNN权重提取每一帧18通道ToF空间特征。"""

    def __init__(self) -> None:
        super().__init__()
        self.network = nn.Sequential(
            nn.Conv2d(TOF_CHANNELS, 32, kernel_size=3, padding=1),
            nn.ReLU(inplace=False),
            nn.Conv2d(32, 48, kernel_size=3, stride=2, padding=1),
            nn.ReLU(inplace=False),
            nn.Conv2d(48, MODEL_DIM, kernel_size=3, stride=2, padding=1),
            nn.ReLU(inplace=False),
            nn.AdaptiveAvgPool2d(1),
        )

    def forward(self, frames: torch.Tensor) -> torch.Tensor:
        """编码ToF帧。

        Args:
            frames: ``[B,18,8,8]``浮点张量，训练前应完成归一化。

        Returns:
            ``[B,64]``空间特征。
        """

        return self.network(frames).flatten(1)


class TinyTofAct(nn.Module):
    """根据12帧ToF和电机历史预测未来10帧三电机目标。

    ``forward``是部署使用的确定性推理路径，潜变量固定为零；
    ``forward_train``额外使用未来动作训练CVAE，但未来动作不会进入推理接口。
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
        self.observation_fusion = nn.Sequential(
            nn.Linear(MODEL_DIM + 16, MODEL_DIM),
            nn.LayerNorm(MODEL_DIM),
            nn.ReLU(inplace=False),
        )
        self.history_position = nn.Parameter(
            torch.zeros(1, HISTORY_FRAMES, MODEL_DIM)
        )

        encoder_layer = nn.TransformerEncoderLayer(
            d_model=MODEL_DIM,
            nhead=ATTENTION_HEADS,
            dim_feedforward=FEEDFORWARD_DIM,
            dropout=MODEL_DROPOUT,
            activation="relu",
            batch_first=True,
            norm_first=False,
        )
        self.temporal_encoder = nn.TransformerEncoder(
            encoder_layer,
            num_layers=ENCODER_LAYERS,
            enable_nested_tensor=False,
        )

        decoder_layer = nn.TransformerDecoderLayer(
            d_model=MODEL_DIM,
            nhead=ATTENTION_HEADS,
            dim_feedforward=FEEDFORWARD_DIM,
            dropout=MODEL_DROPOUT,
            activation="relu",
            batch_first=True,
            norm_first=False,
        )
        self.action_decoder = nn.TransformerDecoder(
            decoder_layer,
            num_layers=DECODER_LAYERS,
        )
        self.action_queries = nn.Parameter(
            torch.zeros(1, ACTION_CHUNK_FRAMES, MODEL_DIM)
        )
        self.latent_projection = nn.Linear(LATENT_DIM, MODEL_DIM)
        self.action_head = nn.Linear(MODEL_DIM, MOTOR_COUNT)

        # 以下编码器仅参与训练，用真实未来动作估计CVAE潜变量。
        self.action_embedding = nn.Linear(MOTOR_COUNT, MODEL_DIM)
        self.action_state_embedding = nn.Linear(MOTOR_COUNT, MODEL_DIM)
        self.action_position = nn.Parameter(
            torch.zeros(1, ACTION_CHUNK_FRAMES + 1, MODEL_DIM)
        )
        action_encoder_layer = nn.TransformerEncoderLayer(
            d_model=MODEL_DIM,
            nhead=ATTENTION_HEADS,
            dim_feedforward=FEEDFORWARD_DIM,
            dropout=MODEL_DROPOUT,
            activation="relu",
            batch_first=True,
            norm_first=False,
        )
        self.action_encoder = nn.TransformerEncoder(
            action_encoder_layer,
            num_layers=1,
            enable_nested_tensor=False,
        )
        self.latent_mean = nn.Linear(MODEL_DIM, LATENT_DIM)
        self.latent_logvar = nn.Linear(MODEL_DIM, LATENT_DIM)

        self._reset_parameters()

    def _reset_parameters(self) -> None:
        """初始化可学习位置和动作查询，避免所有查询完全相同。"""

        nn.init.normal_(self.history_position, std=0.02)
        nn.init.normal_(self.action_position, std=0.02)
        nn.init.normal_(self.action_queries, std=0.02)

    @staticmethod
    def _check_observations(
        tof_history: torch.Tensor,
        state_history: torch.Tensor,
    ) -> None:
        """在进入网络前检查固定输入尺寸，尽早暴露数据接口错误。"""

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

    def encode_observations(
        self,
        tof_history: torch.Tensor,
        state_history: torch.Tensor,
    ) -> torch.Tensor:
        """把空间观测和电机状态融合为12个时序Token。

        Args:
            tof_history: ``[B,12,18,8,8]``。
            state_history: ``[B,12,3]``。

        Returns:
            Transformer编码后的``[B,12,64]``上下文。
        """

        self._check_observations(tof_history, state_history)
        batch_size = tof_history.shape[0]
        frames = tof_history.reshape(
            batch_size * HISTORY_FRAMES,
            TOF_CHANNELS,
            GRID_SIZE,
            GRID_SIZE,
        )
        tof_tokens = self.tof_encoder(frames).reshape(
            batch_size, HISTORY_FRAMES, MODEL_DIM
        )
        state_tokens = self.state_encoder(state_history)
        tokens = self.observation_fusion(
            torch.cat((tof_tokens, state_tokens), dim=-1)
        )
        return self.temporal_encoder(tokens + self.history_position)

    def encode_actions(
        self,
        state_history: torch.Tensor,
        action_chunk: torch.Tensor,
        padding_mask: torch.Tensor,
    ) -> tuple[torch.Tensor, torch.Tensor]:
        """训练时把真实未来动作编码为CVAE均值和对数方差。

        Args:
            state_history: ``[B,12,3]``电机历史。
            action_chunk: ``[B,10,3]``未来动作标签。
            padding_mask: ``[B,10]``，True表示该动作无效且必须忽略。

        Returns:
            ``mean``和``logvar``，尺寸均为``[B,16]``。
        """

        batch_size = state_history.shape[0]
        if tuple(action_chunk.shape) != (
            batch_size,
            ACTION_CHUNK_FRAMES,
            MOTOR_COUNT,
        ):
            raise ValueError("action_chunk尺寸必须为[B,10,3]")
        if tuple(padding_mask.shape) != (batch_size, ACTION_CHUNK_FRAMES):
            raise ValueError("padding_mask尺寸必须为[B,10]")

        state_token = self.action_state_embedding(state_history[:, -1:])
        action_tokens = self.action_embedding(action_chunk)
        tokens = torch.cat((state_token, action_tokens), dim=1)
        tokens = tokens + self.action_position

        state_valid = torch.zeros(
            (batch_size, 1),
            dtype=torch.bool,
            device=padding_mask.device,
        )
        full_mask = torch.cat((state_valid, padding_mask.to(torch.bool)), dim=1)
        encoded = self.action_encoder(tokens, src_key_padding_mask=full_mask)
        latent_token = encoded[:, 0]
        return self.latent_mean(latent_token), self.latent_logvar(latent_token)

    @staticmethod
    def reparameterize(mean: torch.Tensor, logvar: torch.Tensor) -> torch.Tensor:
        """使用重参数化技巧采样训练潜变量，限制方差避免数值溢出。"""

        safe_logvar = torch.clamp(logvar, min=-10.0, max=10.0)
        return mean + torch.randn_like(mean) * torch.exp(0.5 * safe_logvar)

    def decode_actions(self, context: torch.Tensor, latent: torch.Tensor) -> torch.Tensor:
        """使用10个动作查询从观测上下文解码未来三电机目标。"""

        batch_size = context.shape[0]
        queries = self.action_queries.expand(batch_size, -1, -1)
        queries = queries + self.latent_projection(latent).unsqueeze(1)
        decoded = self.action_decoder(tgt=queries, memory=context)
        return self.action_head(decoded)

    def forward(
        self,
        tof_history: torch.Tensor,
        state_history: torch.Tensor,
    ) -> torch.Tensor:
        """执行部署推理，未来动作未知，因此固定使用零潜变量。

        Returns:
            ``[B,10,3]``未来动作序列。
        """

        context = self.encode_observations(tof_history, state_history)
        latent = context.new_zeros((context.shape[0], LATENT_DIM))
        return self.decode_actions(context, latent)

    def forward_train(
        self,
        tof_history: torch.Tensor,
        state_history: torch.Tensor,
        action_chunk: torch.Tensor,
        padding_mask: torch.Tensor,
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """执行训练前向，返回动作预测以及CVAE的均值和对数方差。"""

        context = self.encode_observations(tof_history, state_history)
        mean, logvar = self.encode_actions(
            state_history,
            action_chunk,
            padding_mask,
        )
        latent = self.reparameterize(mean, logvar)
        prediction = self.decode_actions(context, latent)
        return prediction, mean, logvar


def count_parameters(model: nn.Module, trainable_only: bool = True) -> int:
    """统计模型参数量；默认只统计需要梯度的参数。"""

    return sum(
        parameter.numel()
        for parameter in model.parameters()
        if (parameter.requires_grad or not trainable_only)
    )
