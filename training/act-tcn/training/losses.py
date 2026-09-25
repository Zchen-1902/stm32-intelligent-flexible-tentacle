"""Tiny ToF-ACT训练使用的带mask复合损失。"""

from __future__ import annotations

from dataclasses import dataclass

import torch
from torch.nn import functional as F


@dataclass(frozen=True, slots=True)
class ActLossOutput:
    """一次前向得到的总损失和三个可独立观察的分量。"""

    total: torch.Tensor
    action: torch.Tensor
    smooth: torch.Tensor
    kl: torch.Tensor


def _masked_mean(values: torch.Tensor, valid_mask: torch.Tensor) -> torch.Tensor:
    """对有效元素求均值；没有有效元素时直接报错而不是产生NaN。"""

    expanded_mask = valid_mask.expand_as(values)
    valid_count = expanded_mask.sum()
    if int(valid_count.detach().cpu()) == 0:
        raise ValueError("损失中没有有效动作元素")
    return (values * expanded_mask).sum() / valid_count


def compute_act_loss(
    prediction: torch.Tensor,
    target: torch.Tensor,
    padding_mask: torch.Tensor,
    latent_mean: torch.Tensor | None = None,
    latent_logvar: torch.Tensor | None = None,
    smooth_weight: float = 0.10,
    kl_weight: float = 1.0e-4,
) -> ActLossOutput:
    """计算动作、平滑和CVAE KL损失。

    Args:
        prediction: 模型输出``[B,10,3]``。
        target: 归一化动作标签``[B,10,3]``。
        padding_mask: ``[B,10]``，True表示该动作不能参与损失。
        latent_mean/latent_logvar: 训练CVAE时的``[B,latent_dim]``；推理验证可为None。
        smooth_weight: 相邻动作变化误差的权重。
        kl_weight: CVAE KL散度权重。

    Returns:
        包含总损失及各分量的``ActLossOutput``。
    """

    if prediction.shape != target.shape or prediction.ndim != 3:
        raise ValueError("prediction和target必须是相同尺寸的[B,T,3]")
    if padding_mask.shape != prediction.shape[:2]:
        raise ValueError("padding_mask尺寸必须为[B,T]")
    if smooth_weight < 0.0 or kl_weight < 0.0:
        raise ValueError("损失权重不能为负数")

    valid = (~padding_mask.to(torch.bool)).unsqueeze(-1)
    action_error = F.smooth_l1_loss(prediction, target, reduction="none")
    action_loss = _masked_mean(action_error, valid)

    prediction_delta = prediction[:, 1:] - prediction[:, :-1]
    target_delta = target[:, 1:] - target[:, :-1]
    pair_valid = (valid[:, 1:] & valid[:, :-1])
    if bool(pair_valid.any()):
        smooth_error = F.smooth_l1_loss(
            prediction_delta,
            target_delta,
            reduction="none",
        )
        smooth_loss = _masked_mean(smooth_error, pair_valid)
    else:
        smooth_loss = prediction.sum() * 0.0

    if (latent_mean is None) != (latent_logvar is None):
        raise ValueError("latent_mean和latent_logvar必须同时提供或同时省略")
    if latent_mean is None:
        kl_loss = prediction.sum() * 0.0
    else:
        if latent_mean.shape != latent_logvar.shape:
            raise ValueError("latent_mean和latent_logvar尺寸必须一致")
        safe_logvar = torch.clamp(latent_logvar, min=-10.0, max=10.0)
        kl_loss = -0.5 * torch.mean(
            1.0 + safe_logvar - latent_mean.square() - safe_logvar.exp()
        )

    total = action_loss + smooth_weight * smooth_loss + kl_weight * kl_loss
    return ActLossOutput(total, action_loss, smooth_loss, kl_loss)
