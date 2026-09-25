"""在独立测试episode上评估Tiny ToF-ACT模型。"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import torch
from torch.utils.data import DataLoader

from training.dataset import EpisodeWindowDataset
from training.model import TinyTofAct
from training.normalization import NormalizationStats
from training.train import move_batch


def _resolve_device(name: str) -> torch.device:
    """根据命令行参数选择CPU或CUDA，并拒绝不可用的CUDA请求。"""

    if name == "cuda" and not torch.cuda.is_available():
        raise RuntimeError("请求使用CUDA，但当前PyTorch未检测到CUDA")
    if name == "auto":
        return torch.device("cuda" if torch.cuda.is_available() else "cpu")
    return torch.device(name)


def _load_test_paths(run_dir: Path) -> list[Path]:
    """从训练时固定保存的split.json读取测试episode，防止重新随机划分。"""

    split = json.loads((run_dir / "split.json").read_text(encoding="utf-8"))
    paths = [Path(path) for path in split.get("test", [])]
    if not paths:
        raise ValueError("split.json中没有测试episode")
    return paths


@torch.no_grad()
def evaluate_model(
    model: TinyTofAct,
    loader: DataLoader,
    stats: NormalizationStats,
    device: torch.device,
) -> dict[str, object]:
    """按有效动作mask计算原始q单位下的测试指标。

    Args:
        model: 已加载checkpoint且处于当前固定结构的Tiny ToF-ACT。
        loader: 使用同一stats归一化后的测试DataLoader。
        stats: 训练集统计量，用于把预测、标签和状态还原为原始q。
        device: 模型实际运行设备。

    Returns:
        总体及每台电机MAE、rad MAE、方向正确率和动作平滑度。

    Note:
        方向正确率只统计标签相对当前位置变化超过1q的动作，避免静止点符号无意义。
    """

    model.eval()
    action_mean = torch.as_tensor(stats.action_mean, device=device)
    action_std = torch.as_tensor(stats.action_std, device=device)
    state_mean = torch.as_tensor(stats.state_mean, device=device)
    state_std = torch.as_tensor(stats.state_std, device=device)

    abs_sum = torch.zeros(3, dtype=torch.float64, device=device)
    valid_count = torch.zeros(3, dtype=torch.float64, device=device)
    direction_correct = torch.zeros(3, dtype=torch.float64, device=device)
    direction_count = torch.zeros(3, dtype=torch.float64, device=device)
    smooth_sum = torch.zeros(3, dtype=torch.float64, device=device)
    smooth_count = torch.zeros(3, dtype=torch.float64, device=device)

    for raw_batch in loader:
        batch = move_batch(raw_batch, device)
        prediction = model(batch["tof_history"], batch["state_history"])
        prediction_q = prediction * action_std + action_mean
        target_q = batch["action_chunk"] * action_std + action_mean
        current_q = batch["state_history"][:, -1] * state_std + state_mean
        valid = (~batch["padding_mask"]).unsqueeze(-1).expand_as(prediction_q)

        abs_sum += (torch.abs(prediction_q - target_q) * valid).sum((0, 1)).double()
        valid_count += valid.sum((0, 1)).double()

        target_delta = target_q - current_q.unsqueeze(1)
        prediction_delta = prediction_q - current_q.unsqueeze(1)
        moving = valid & (torch.abs(target_delta) > 1.0)
        direction_correct += (
            (torch.sign(prediction_delta) == torch.sign(target_delta)) & moving
        ).sum((0, 1)).double()
        direction_count += moving.sum((0, 1)).double()

        pair_valid = valid[:, 1:] & valid[:, :-1]
        smooth_sum += (
            torch.abs(prediction_q[:, 1:] - prediction_q[:, :-1]) * pair_valid
        ).sum((0, 1)).double()
        smooth_count += pair_valid.sum((0, 1)).double()

    if bool((valid_count == 0).any()):
        raise ValueError("测试集中至少一台电机没有有效标签")

    mae_q = (abs_sum / valid_count).cpu().numpy()
    direction = torch.where(
        direction_count > 0,
        direction_correct / direction_count,
        torch.zeros_like(direction_count),
    ).cpu().numpy()
    smooth_q = torch.where(
        smooth_count > 0,
        smooth_sum / smooth_count,
        torch.zeros_like(smooth_count),
    ).cpu().numpy()
    total_mae_q = float(abs_sum.sum().cpu() / valid_count.sum().cpu())

    return {
        "mae_q": total_mae_q,
        "mae_rad": total_mae_q * 0.01,
        "motor_mae_q": mae_q.tolist(),
        "motor_mae_rad": (mae_q * 0.01).tolist(),
        "motor_direction_accuracy": direction.tolist(),
        "motor_prediction_smooth_q": smooth_q.tolist(),
        "valid_action_values": int(valid_count.sum().cpu()),
    }


def build_argument_parser() -> argparse.ArgumentParser:
    """构建测试命令参数。"""

    parser = argparse.ArgumentParser(description="评估Tiny ToF-ACT测试集")
    parser.add_argument("--run-dir", type=Path, required=True, help="训练输出目录")
    parser.add_argument("--checkpoint", type=Path, default=None, help="默认使用best.pt")
    parser.add_argument("--batch-size", type=int, default=32)
    parser.add_argument("--num-workers", type=int, default=0)
    parser.add_argument("--device", default="auto", choices=("auto", "cpu", "cuda"))
    return parser


def main() -> None:
    """加载固定测试集和最佳权重，打印并保存metrics.json。"""

    args = build_argument_parser().parse_args()
    if args.batch_size <= 0:
        raise ValueError("batch-size必须大于0")
    device = _resolve_device(args.device)
    checkpoint_path = args.checkpoint or (args.run_dir / "best.pt")
    stats = NormalizationStats.load(args.run_dir / "stats.json")
    dataset = EpisodeWindowDataset(_load_test_paths(args.run_dir), stats)
    loader = DataLoader(
        dataset,
        batch_size=args.batch_size,
        shuffle=False,
        num_workers=args.num_workers,
        pin_memory=device.type == "cuda",
    )

    checkpoint = torch.load(checkpoint_path, map_location=device, weights_only=False)
    model = TinyTofAct().to(device)
    model.load_state_dict(checkpoint["model_state"])
    metrics = evaluate_model(model, loader, stats, device)
    metrics["checkpoint"] = str(checkpoint_path.resolve())
    metrics["test_episodes"] = len(dataset.episode_paths)
    metrics["test_windows"] = len(dataset)
    (args.run_dir / "metrics.json").write_text(
        json.dumps(metrics, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    print(json.dumps(metrics, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
