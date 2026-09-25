"""将确定性Tiny ToF-ACT推理路径导出为固定尺寸ONNX。"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import torch
from torch import nn

from training.config import (
    ACTION_CHUNK_FRAMES,
    GRID_SIZE,
    HISTORY_FRAMES,
    MOTOR_COUNT,
    TOF_CHANNELS,
)
from training.model import TinyTofAct
from training.normalization import NormalizationStats


class DeploymentModel(nn.Module):
    """封装归一化、Tiny ToF-ACT和动作反归一化。

    Cube.AI只需输入原始18通道ToF历史和三电机实际位置历史，输出单位直接为
    0.01rad机械角。统计量作为模型常量写进ONNX，不需要在H7重复维护。
    """

    def __init__(self, model: TinyTofAct, stats: NormalizationStats) -> None:
        super().__init__()
        self.model = model
        self.register_buffer(
            "tof_mean",
            torch.as_tensor(stats.tof_mean).view(1, 1, TOF_CHANNELS, 1, 1),
        )
        self.register_buffer(
            "tof_std",
            torch.as_tensor(stats.tof_std).view(1, 1, TOF_CHANNELS, 1, 1),
        )
        self.register_buffer(
            "state_mean",
            torch.as_tensor(stats.state_mean).view(1, 1, MOTOR_COUNT),
        )
        self.register_buffer(
            "state_std",
            torch.as_tensor(stats.state_std).view(1, 1, MOTOR_COUNT),
        )
        self.register_buffer(
            "action_mean",
            torch.as_tensor(stats.action_mean).view(1, 1, MOTOR_COUNT),
        )
        self.register_buffer(
            "action_std",
            torch.as_tensor(stats.action_std).view(1, 1, MOTOR_COUNT),
        )

    def forward(
        self,
        tof_history: torch.Tensor,
        state_history: torch.Tensor,
    ) -> torch.Tensor:
        """输入原始数据并输出``[1,10,3]``未来目标q。"""

        normalized_tof = (tof_history - self.tof_mean) / self.tof_std
        normalized_state = (state_history - self.state_mean) / self.state_std
        normalized_action = self.model(normalized_tof, normalized_state)
        return normalized_action * self.action_std + self.action_mean


def build_argument_parser() -> argparse.ArgumentParser:
    """构建ONNX导出命令参数。"""

    parser = argparse.ArgumentParser(description="导出Tiny ToF-ACT ONNX")
    parser.add_argument("--run-dir", type=Path, required=True, help="训练输出目录")
    parser.add_argument("--checkpoint", type=Path, default=None, help="默认使用best.pt")
    parser.add_argument("--output", type=Path, default=None, help="默认写入run-dir/model.onnx")
    parser.add_argument("--opset", type=int, default=17)
    return parser


def main() -> None:
    """加载最佳权重并导出Cube.AI使用的固定batch=1模型。"""

    args = build_argument_parser().parse_args()
    checkpoint_path = args.checkpoint or (args.run_dir / "best.pt")
    output_path = args.output or (args.run_dir / "model.onnx")
    if args.opset < 17:
        raise ValueError("Transformer导出要求opset不低于17")

    stats = NormalizationStats.load(args.run_dir / "stats.json")
    checkpoint = torch.load(checkpoint_path, map_location="cpu", weights_only=False)
    model = TinyTofAct()
    model.load_state_dict(checkpoint["model_state"])
    deployment_model = DeploymentModel(model.eval(), stats).eval()

    # 固定输入尺寸可减少H7部署图中的动态Shape算子和额外内存。
    tof_example = torch.zeros(
        1,
        HISTORY_FRAMES,
        TOF_CHANNELS,
        GRID_SIZE,
        GRID_SIZE,
        dtype=torch.float32,
    )
    state_example = torch.zeros(
        1,
        HISTORY_FRAMES,
        MOTOR_COUNT,
        dtype=torch.float32,
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    torch.onnx.export(
        deployment_model,
        (tof_example, state_example),
        output_path,
        input_names=("tof_history", "state_history"),
        output_names=("action_q",),
        opset_version=args.opset,
        do_constant_folding=True,
        dynamo=False,
    )

    metadata = {
        "version": 1,
        "checkpoint": str(checkpoint_path.resolve()),
        "inputs": {
            "tof_history": [1, HISTORY_FRAMES, TOF_CHANNELS, GRID_SIZE, GRID_SIZE],
            "state_history": [1, HISTORY_FRAMES, MOTOR_COUNT],
        },
        "output": {"action_q": [1, ACTION_CHUNK_FRAMES, MOTOR_COUNT]},
        "action_unit": "0.01rad mechanical angle",
        "normalization_embedded": True,
        "opset": args.opset,
    }
    output_path.with_suffix(".json").write_text(
        json.dumps(metadata, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    print(f"ONNX已导出: {output_path}")


if __name__ == "__main__":
    main()
