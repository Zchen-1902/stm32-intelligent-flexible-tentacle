"""Tiny ToF-ACT训练、验证和checkpoint保存入口。"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import random

import numpy as np
import torch
from torch.utils.data import DataLoader

from training.dataset import EpisodeWindowDataset
from training.losses import compute_act_loss
from training.model import TinyTofAct, count_parameters
from training.normalization import fit_normalization


def discover_episodes(data_dir: str | Path) -> list[Path]:
    """递归查找上位机保存的NPZ episode并按路径排序。"""

    paths = sorted(Path(data_dir).rglob("*.npz"))
    if not paths:
        raise ValueError(f"{data_dir}中没有找到episode .npz文件")
    return paths


def split_episodes(
    paths: list[Path],
    seed: int,
    train_ratio: float = 0.70,
    val_ratio: float = 0.15,
) -> tuple[list[Path], list[Path], list[Path]]:
    """按完整episode划分训练、验证和测试集。

    Note:
        至少需要3条episode，保证三组均非空。同一episode不会跨组出现。
    """

    if len(paths) < 3:
        raise ValueError("至少需要3条episode才能划分训练/验证/测试集")
    if not (0.0 < train_ratio < 1.0 and 0.0 < val_ratio < 1.0):
        raise ValueError("train_ratio和val_ratio必须位于0..1")

    shuffled = list(paths)
    random.Random(seed).shuffle(shuffled)
    train_count = max(1, int(len(shuffled) * train_ratio))
    val_count = max(1, int(len(shuffled) * val_ratio))
    if train_count + val_count >= len(shuffled):
        train_count = len(shuffled) - 2
        val_count = 1

    train_paths = shuffled[:train_count]
    val_paths = shuffled[train_count : train_count + val_count]
    test_paths = shuffled[train_count + val_count :]
    return train_paths, val_paths, test_paths


def set_random_seed(seed: int) -> None:
    """固定Python、NumPy和PyTorch随机种子，便于重复实验。"""

    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)


def move_batch(batch: dict[str, torch.Tensor], device: torch.device) -> dict[str, torch.Tensor]:
    """把一个Dataset batch移动到训练设备并固定所需类型。"""

    return {
        "tof_history": batch["tof_history"].to(device=device, dtype=torch.float32),
        "state_history": batch["state_history"].to(device=device, dtype=torch.float32),
        "action_chunk": batch["action_chunk"].to(device=device, dtype=torch.float32),
        "padding_mask": batch["padding_mask"].to(device=device, dtype=torch.bool),
    }


def train_one_epoch(
    model: TinyTofAct,
    loader: DataLoader,
    optimizer: torch.optim.Optimizer,
    device: torch.device,
    smooth_weight: float,
    kl_weight: float,
) -> dict[str, float]:
    """训练一个epoch并返回按样本数加权的损失均值。"""

    model.train()
    totals = {"total": 0.0, "action": 0.0, "smooth": 0.0, "kl": 0.0}
    sample_count = 0

    for raw_batch in loader:
        batch = move_batch(raw_batch, device)
        optimizer.zero_grad(set_to_none=True)
        prediction, mean, logvar = model.forward_train(
            batch["tof_history"],
            batch["state_history"],
            batch["action_chunk"],
            batch["padding_mask"],
        )
        loss = compute_act_loss(
            prediction,
            batch["action_chunk"],
            batch["padding_mask"],
            mean,
            logvar,
            smooth_weight,
            kl_weight,
        )
        loss.total.backward()
        torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
        optimizer.step()

        batch_size = prediction.shape[0]
        sample_count += batch_size
        totals["total"] += float(loss.total.detach()) * batch_size
        totals["action"] += float(loss.action.detach()) * batch_size
        totals["smooth"] += float(loss.smooth.detach()) * batch_size
        totals["kl"] += float(loss.kl.detach()) * batch_size

    return {name: value / sample_count for name, value in totals.items()}


@torch.no_grad()
def validate(
    model: TinyTofAct,
    loader: DataLoader,
    device: torch.device,
    smooth_weight: float,
) -> dict[str, float]:
    """使用与H7一致的零潜变量推理路径计算验证损失。"""

    model.eval()
    totals = {"total": 0.0, "action": 0.0, "smooth": 0.0}
    sample_count = 0
    for raw_batch in loader:
        batch = move_batch(raw_batch, device)
        prediction = model(batch["tof_history"], batch["state_history"])
        loss = compute_act_loss(
            prediction,
            batch["action_chunk"],
            batch["padding_mask"],
            smooth_weight=smooth_weight,
            kl_weight=0.0,
        )
        batch_size = prediction.shape[0]
        sample_count += batch_size
        totals["total"] += float(loss.total) * batch_size
        totals["action"] += float(loss.action) * batch_size
        totals["smooth"] += float(loss.smooth) * batch_size
    return {name: value / sample_count for name, value in totals.items()}


def build_argument_parser() -> argparse.ArgumentParser:
    """构建命令行参数，所有输出都写入独立run目录。"""

    parser = argparse.ArgumentParser(description="训练Tiny ToF-ACT")
    parser.add_argument("--data-dir", type=Path, required=True, help="episode .npz目录")
    parser.add_argument("--output-dir", type=Path, required=True, help="训练输出目录")
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--batch-size", type=int, default=32)
    parser.add_argument("--learning-rate", type=float, default=1.0e-3)
    parser.add_argument("--weight-decay", type=float, default=1.0e-4)
    parser.add_argument("--smooth-weight", type=float, default=0.10)
    parser.add_argument("--kl-weight", type=float, default=1.0e-4)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--num-workers", type=int, default=0)
    parser.add_argument("--device", default="auto", choices=("auto", "cpu", "cuda"))
    return parser


def main() -> None:
    """执行episode划分、归一化、训练验证及best/last权重保存。"""

    args = build_argument_parser().parse_args()
    if args.epochs <= 0 or args.batch_size <= 0:
        raise ValueError("epochs和batch-size必须大于0")
    set_random_seed(args.seed)

    if args.device == "cuda" and not torch.cuda.is_available():
        raise RuntimeError("请求使用CUDA，但当前PyTorch未检测到CUDA")
    device = torch.device(
        "cuda" if args.device == "auto" and torch.cuda.is_available() else
        ("cpu" if args.device == "auto" else args.device)
    )

    episode_paths = discover_episodes(args.data_dir)
    train_paths, val_paths, test_paths = split_episodes(episode_paths, args.seed)
    args.output_dir.mkdir(parents=True, exist_ok=True)

    split_payload = {
        "seed": args.seed,
        "train": [str(path.resolve()) for path in train_paths],
        "validation": [str(path.resolve()) for path in val_paths],
        "test": [str(path.resolve()) for path in test_paths],
    }
    (args.output_dir / "split.json").write_text(
        json.dumps(split_payload, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )

    stats = fit_normalization(train_paths)
    stats.save(args.output_dir / "stats.json")
    train_dataset = EpisodeWindowDataset(train_paths, stats)
    val_dataset = EpisodeWindowDataset(val_paths, stats)
    pin_memory = device.type == "cuda"
    train_loader = DataLoader(
        train_dataset,
        batch_size=args.batch_size,
        shuffle=True,
        num_workers=args.num_workers,
        pin_memory=pin_memory,
    )
    val_loader = DataLoader(
        val_dataset,
        batch_size=args.batch_size,
        shuffle=False,
        num_workers=args.num_workers,
        pin_memory=pin_memory,
    )

    model = TinyTofAct().to(device)
    optimizer = torch.optim.AdamW(
        model.parameters(),
        lr=args.learning_rate,
        weight_decay=args.weight_decay,
    )
    history: list[dict[str, object]] = []
    best_val = float("inf")

    print(
        f"device={device} parameters={count_parameters(model)} "
        f"episodes(train/val/test)={len(train_paths)}/{len(val_paths)}/{len(test_paths)} "
        f"windows(train/val)={len(train_dataset)}/{len(val_dataset)}"
    )

    for epoch in range(1, args.epochs + 1):
        train_metrics = train_one_epoch(
            model,
            train_loader,
            optimizer,
            device,
            args.smooth_weight,
            args.kl_weight,
        )
        val_metrics = validate(
            model,
            val_loader,
            device,
            args.smooth_weight,
        )
        record = {"epoch": epoch, "train": train_metrics, "validation": val_metrics}
        history.append(record)
        checkpoint = {
            "epoch": epoch,
            "model_state": model.state_dict(),
            "optimizer_state": optimizer.state_dict(),
            "validation_loss": val_metrics["total"],
            "parameter_count": count_parameters(model),
        }
        torch.save(checkpoint, args.output_dir / "last.pt")
        if val_metrics["total"] < best_val:
            best_val = val_metrics["total"]
            torch.save(checkpoint, args.output_dir / "best.pt")

        (args.output_dir / "history.json").write_text(
            json.dumps(history, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        print(
            f"epoch={epoch:03d} train={train_metrics['total']:.6f} "
            f"val={val_metrics['total']:.6f} best={best_val:.6f}"
        )


if __name__ == "__main__":
    main()
