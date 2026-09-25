#!/usr/bin/env python3
"""Train a compact CNN to regress hand openness from VL53 8x8 CNH CSV data."""

from __future__ import annotations

import argparse
import csv
import json
import math
import random
from pathlib import Path

import numpy as np
import torch
from torch import nn
from torch.utils.data import DataLoader, TensorDataset

from train_openness import (
    CNH_BINS,
    LABEL_NONE,
    VALID_TARGET_STATUS,
    ZONE_COUNT,
    build_column_names,
    evaluate,
    evaluate_by_group,
    require_columns,
    split_indices,
    z_group_name,
)


GRID_SIZE = 8
CNN_CHANNELS = 20
DISTANCE_MIN_MM = 160.0
DISTANCE_MAX_MM = 450.0
FOREGROUND_MARGIN_MM = 120.0
REL_DISTANCE_SCALE_MM = 200.0
CNH_EPS = 1e-6
Z_CENTER_MM = 300.0
Z_SCALE_MM = 150.0


class OpennessCNN(nn.Module):
    """小型CNN回归网络：输入20x8x8特征图，输出0~1的张握程度。"""

    def __init__(self, channels: int = CNN_CHANNELS, dense_hidden: int = 24) -> None:
        super().__init__()
        self.net = nn.Sequential(
            nn.Conv2d(channels, 24, kernel_size=3, padding=1),
            nn.ReLU(),
            nn.Conv2d(24, 32, kernel_size=3, padding=1),
            nn.ReLU(),
            nn.Conv2d(32, 32, kernel_size=3, padding=1),
            nn.ReLU(),
            nn.AdaptiveAvgPool2d((1, 1)),
            nn.Flatten(),
            nn.Linear(32, dense_hidden),
            nn.ReLU(),
            nn.Linear(dense_hidden, 1),
            nn.Sigmoid(),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.net(x)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Train VL53 openness CNN regression model.")
    parser.add_argument("--csv", required=True, type=Path, help="Path to gesture.csv.")
    parser.add_argument("--out", type=Path, default=Path("H743VIT6/Training/exported_cnn"), help="Output directory.")
    parser.add_argument("--epochs", type=int, default=300)
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--lr", type=float, default=1e-3)
    parser.add_argument("--val-ratio", type=float, default=0.2)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--dense-hidden", type=int, default=24)
    parser.add_argument("--huber-beta", type=float, default=0.10, help="Huber转折点，0.10约等于10分误差。")
    parser.add_argument("--no-sample-weights", action="store_true", help="关闭距离x标签加权。")
    parser.add_argument("--weight-clip", type=float, default=3.0, help="样本权重上限，避免极少数组过度放大。")
    parser.add_argument("--min-valid-zones", type=int, default=3, help="Drop rows with fewer valid foreground zones.")
    parser.add_argument("--no-quality-filter", action="store_true", help="Do not filter by target_status/nb_target_detected.")
    parser.add_argument(
        "--val-split",
        choices=["random", "z-holdout"],
        default="random",
        help="random=普通随机验证；z-holdout=按手部距离段留出验证。",
    )
    parser.add_argument("--eval-both-splits", action="store_true", help="依次训练 random 和 z-holdout 两套报告。")
    return parser.parse_args()


def build_channel_names() -> list[str]:
    """返回CNN每个通道的固定含义，部署前处理必须保持同一顺序。"""

    return [
        "rel_distance",
        "abs_distance_norm",
        "valid_mask",
        "foreground_mask",
        "target_status_ok",
        "nb_target_norm",
        *[f"cnh_bin_{idx}" for idx in range(CNH_BINS)],
        "center_x_norm",
        "center_y_norm",
        "center_z_norm",
        "foreground_zone_count_norm",
        "bbox_width_norm",
        "bbox_height_norm",
    ]


def preprocess_frame_cnn(
    distance: np.ndarray,
    target_status: np.ndarray,
    nb_target: np.ndarray,
    cnh: np.ndarray,
    min_valid_zones: int,
    quality_filter: bool,
) -> tuple[np.ndarray, dict, str | None]:
    """把一帧CSV数据转成固定20x8x8 CNN输入。

    这里和当前MCU前处理保持同一组前景参数：有效距离160~450mm，
    前景区域为 d_min + 120mm。手臂干扰后续优先靠均衡采样和CNN空间特征处理。
    """

    valid_mask = (distance >= DISTANCE_MIN_MM) & (distance <= DISTANCE_MAX_MM)
    status_ok = np.isin(target_status, VALID_TARGET_STATUS)
    nb_norm = np.clip(nb_target.astype(np.float32), 0.0, 3.0) / 3.0
    if quality_filter:
        valid_mask &= status_ok & (nb_target > 0)

    valid_count = int(np.sum(valid_mask))
    if valid_count < min_valid_zones:
        return np.empty((0,), dtype=np.float32), {}, "few_valid_zones"

    d_min = float(np.min(distance[valid_mask]))
    foreground_mask = valid_mask & (distance <= d_min + FOREGROUND_MARGIN_MM)
    foreground_count = int(np.sum(foreground_mask))
    if foreground_count < min_valid_zones:
        return np.empty((0,), dtype=np.float32), {}, "few_foreground_zones"

    zones = np.flatnonzero(foreground_mask)
    xs = zones % GRID_SIZE
    ys = zones // GRID_SIZE
    center_z = float(np.median(distance[foreground_mask]))
    weights = 1.0 / np.maximum(distance[foreground_mask], 1.0)
    weight_sum = float(np.sum(weights))
    center_x = float(np.sum(xs * weights) / weight_sum)
    center_y = float(np.sum(ys * weights) / weight_sum)
    bbox_width = int(np.max(xs) - np.min(xs) + 1)
    bbox_height = int(np.max(ys) - np.min(ys) + 1)

    feature = np.zeros((CNN_CHANNELS, GRID_SIZE, GRID_SIZE), dtype=np.float32)
    dist_grid = distance.reshape(GRID_SIZE, GRID_SIZE)
    valid_grid = valid_mask.reshape(GRID_SIZE, GRID_SIZE)
    fg_grid = foreground_mask.reshape(GRID_SIZE, GRID_SIZE)

    rel = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.float32)
    rel[fg_grid] = np.clip((dist_grid[fg_grid] - center_z) / REL_DISTANCE_SCALE_MM, -3.0, 3.0)
    feature[0] = rel

    abs_norm = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.float32)
    abs_norm[fg_grid] = np.clip(
        (dist_grid[fg_grid] - DISTANCE_MIN_MM) / (DISTANCE_MAX_MM - DISTANCE_MIN_MM),
        0.0,
        1.0,
    )
    feature[1] = abs_norm
    feature[2] = valid_grid.astype(np.float32)
    feature[3] = fg_grid.astype(np.float32)
    feature[4] = status_ok.reshape(GRID_SIZE, GRID_SIZE).astype(np.float32)
    feature[5] = nb_norm.reshape(GRID_SIZE, GRID_SIZE)

    cnh_matrix = cnh.reshape(ZONE_COUNT, CNH_BINS).astype(np.float32)
    cnh_feature = np.zeros_like(cnh_matrix)
    cnh_feature[foreground_mask, :] = cnh_matrix[foreground_mask, :]
    cnh_scale = float(np.max(np.abs(cnh_feature[foreground_mask, :])))
    if cnh_scale < CNH_EPS:
        cnh_scale = 1.0
    cnh_feature /= cnh_scale
    for bin_idx in range(CNH_BINS):
        feature[6 + bin_idx] = cnh_feature[:, bin_idx].reshape(GRID_SIZE, GRID_SIZE)

    feature[14, :, :] = np.clip((center_x - 3.5) / 3.5, -1.0, 1.0)
    feature[15, :, :] = np.clip((center_y - 3.5) / 3.5, -1.0, 1.0)
    feature[16, :, :] = np.clip((center_z - Z_CENTER_MM) / Z_SCALE_MM, -2.0, 2.0)
    feature[17, :, :] = foreground_count / float(ZONE_COUNT)
    feature[18, :, :] = bbox_width / float(GRID_SIZE)
    feature[19, :, :] = bbox_height / float(GRID_SIZE)

    frame_meta = {
        "center_x": center_x,
        "center_y": center_y,
        "center_z": center_z,
        "hand_zone_count": foreground_count,
        "bbox_width": bbox_width,
        "bbox_height": bbox_height,
        "z_bin": int(np.digitize(center_z, [220.0, 280.0, 340.0, 420.0])),
    }
    return feature, frame_meta, None


def read_dataset_cnn(csv_path: Path, min_valid_zones: int, quality_filter: bool) -> tuple[np.ndarray, np.ndarray, dict]:
    """读取CSV并生成CNN训练张量。

    输出x形状为 N x 20 x 8 x 8，标签y为0~1。target_status和nb_target
    同时参与质量过滤和CNN输入，帮助模型理解哪些zone可靠。
    """

    distance_cols, status_cols, nb_cols, cnh_cols = build_column_names()
    base_cols = ["timestamp_ms", "label_score", "frame_count", "mode", "valid"]

    rows: list[np.ndarray] = []
    labels: list[float] = []
    sample_meta: list[dict] = []
    label_hist: dict[str, int] = {}
    z_group_hist: dict[str, int] = {}
    dropped = {
        "label_none": 0,
        "bad_label": 0,
        "invalid_frame": 0,
        "few_valid_zones": 0,
        "few_foreground_zones": 0,
        "parse_error": 0,
    }

    with csv_path.open("r", newline="", encoding="utf-8-sig") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            raise ValueError("CSV has no header.")
        require_columns(reader.fieldnames, base_cols + distance_cols + status_cols + nb_cols + cnh_cols)

        for row in reader:
            try:
                label = int(row["label_score"])
                valid = int(row["valid"])
                if label == LABEL_NONE:
                    dropped["label_none"] += 1
                    continue
                if label < 0 or label > 100:
                    dropped["bad_label"] += 1
                    continue
                if valid != 1:
                    dropped["invalid_frame"] += 1
                    continue

                distance = np.array([float(row[name]) for name in distance_cols], dtype=np.float32)
                target_status = np.array([int(row[name]) for name in status_cols], dtype=np.int16)
                nb_target = np.array([int(row[name]) for name in nb_cols], dtype=np.int16)
                cnh = np.array([float(row[name]) for name in cnh_cols], dtype=np.float32)
                feature, frame_meta, drop_reason = preprocess_frame_cnn(
                    distance,
                    target_status,
                    nb_target,
                    cnh,
                    min_valid_zones,
                    quality_filter,
                )
                if drop_reason is not None:
                    dropped[drop_reason] += 1
                    continue
            except (TypeError, ValueError, KeyError):
                dropped["parse_error"] += 1
                continue

            rows.append(feature)
            labels.append(label / 100.0)
            sample_meta.append(frame_meta)
            label_key = str(label)
            label_hist[label_key] = label_hist.get(label_key, 0) + 1
            z_key = z_group_name(int(frame_meta["z_bin"]))
            z_group_hist[z_key] = z_group_hist.get(z_key, 0) + 1

    if not rows:
        raise ValueError("No usable training rows after filtering.")

    x = np.asarray(rows, dtype=np.float32)
    y = np.asarray(labels, dtype=np.float32).reshape(-1, 1)
    meta = {
        "channel_names": build_channel_names(),
        "sample_meta": sample_meta,
        "z_group_hist": dict(sorted(z_group_hist.items())),
        "label_hist": dict(sorted(label_hist.items(), key=lambda item: int(item[0]))),
        "dropped": dropped,
        "total_used": int(x.shape[0]),
        "input_shape": [CNN_CHANNELS, GRID_SIZE, GRID_SIZE],
        "preprocess": {
            "mode": "cnn_20ch_foreground_relative",
            "distance_min_mm": DISTANCE_MIN_MM,
            "distance_max_mm": DISTANCE_MAX_MM,
            "foreground_margin_mm": FOREGROUND_MARGIN_MM,
            "relative_distance_scale_mm": REL_DISTANCE_SCALE_MM,
            "cnh_normalization": "per_frame_foreground_abs_max",
            "normalization": "per_channel_train_mean_std",
        },
    }
    return x, y, meta


def normalize_train_val(x_train: np.ndarray, x_val: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """按通道归一化，避免MCU端保存1280个均值/方差。"""

    mean = x_train.mean(axis=(0, 2, 3), keepdims=True)
    std = x_train.std(axis=(0, 2, 3), keepdims=True)
    std = np.where(std < 1e-6, 1.0, std)
    return (x_train - mean) / std, (x_val - mean) / std, mean.reshape(-1).astype(np.float32), std.reshape(-1).astype(np.float32)


def build_distance_label_hist(
    labels: np.ndarray,
    sample_meta: list[dict],
    indices: np.ndarray | None = None,
) -> dict[str, dict[str, int]]:
    """统计距离段x标签数量，用于检查数据是否均衡。"""

    hist: dict[str, dict[str, int]] = {}
    used_indices = np.arange(labels.shape[0]) if indices is None else indices

    for idx_raw in used_indices:
        idx = int(idx_raw)
        z_key = z_group_name(int(sample_meta[idx]["z_bin"]))
        label_key = str(int(round(float(labels[idx, 0]) * 100.0)))
        if z_key not in hist:
            hist[z_key] = {}
        hist[z_key][label_key] = hist[z_key].get(label_key, 0) + 1

    return {
        z_key: dict(sorted(label_counts.items(), key=lambda item: int(item[0])))
        for z_key, label_counts in sorted(hist.items())
    }


def build_sample_weights(
    labels: np.ndarray,
    sample_meta: list[dict],
    train_idx: np.ndarray,
    clip_max: float,
) -> np.ndarray:
    """按距离段x标签构造样本权重，缓解少样本距离/标签被主流样本淹没。

    权重使用 1/sqrt(count)，再归一化到均值为1，并做上限裁剪。
    这样少样本组合会被增强，但不会因为极少数组导致训练发散。
    """

    counts: dict[tuple[str, str], int] = {}
    keys: list[tuple[str, str]] = []

    for idx_raw in train_idx:
        idx = int(idx_raw)
        z_key = z_group_name(int(sample_meta[idx]["z_bin"]))
        label_key = str(int(round(float(labels[idx, 0]) * 100.0)))
        key = (z_key, label_key)
        keys.append(key)
        counts[key] = counts.get(key, 0) + 1

    weights = np.asarray([1.0 / math.sqrt(float(counts[key])) for key in keys], dtype=np.float32)
    mean_weight = float(np.mean(weights))
    if mean_weight > 0.0:
        weights /= mean_weight

    if clip_max > 0.0:
        weights = np.clip(weights, 1.0 / clip_max, clip_max)

    return weights.reshape(-1, 1).astype(np.float32)


def weighted_huber_loss(
    pred: torch.Tensor,
    target: torch.Tensor,
    weight: torch.Tensor,
    beta: float,
) -> torch.Tensor:
    """加权Huber损失，标签已归一化到0~1。"""

    beta_value = max(float(beta), 1e-6)
    diff = torch.abs(pred - target)
    quadratic = torch.clamp(diff, max=beta_value)
    linear = diff - quadratic
    loss = (0.5 * quadratic * quadratic / beta_value) + linear

    return torch.sum(loss * weight) / torch.clamp(torch.sum(weight), min=1.0)


def train_model(args: argparse.Namespace) -> None:
    random.seed(args.seed)
    np.random.seed(args.seed)
    torch.manual_seed(args.seed)

    x, y, meta = read_dataset_cnn(args.csv, args.min_valid_zones, quality_filter=not args.no_quality_filter)
    train_idx, val_idx, split_meta = split_indices(x.shape[0], args.val_ratio, args.seed, meta["sample_meta"], args.val_split)
    x_train, y_train = x[train_idx], y[train_idx]
    x_val, y_val = x[val_idx], y[val_idx]
    x_train_n, x_val_n, channel_mean, channel_std = normalize_train_val(x_train, x_val)
    train_weights = (
        np.ones_like(y_train, dtype=np.float32)
        if args.no_sample_weights
        else build_sample_weights(y, meta["sample_meta"], train_idx, args.weight_clip)
    )

    train_ds = TensorDataset(
        torch.from_numpy(x_train_n),
        torch.from_numpy(y_train),
        torch.from_numpy(train_weights),
    )
    train_loader = DataLoader(train_ds, batch_size=args.batch_size, shuffle=True)
    x_val_t = torch.from_numpy(x_val_n)
    y_val_t = torch.from_numpy(y_val)

    model = OpennessCNN(channels=CNN_CHANNELS, dense_hidden=args.dense_hidden)
    optimizer = torch.optim.Adam(model.parameters(), lr=args.lr)
    best_state = None
    best_mae = math.inf
    best_epoch = 0

    for epoch in range(1, args.epochs + 1):
        model.train()
        train_loss_sum = 0.0
        train_count = 0
        for xb, yb, wb in train_loader:
            optimizer.zero_grad(set_to_none=True)
            pred = model(xb)
            loss = weighted_huber_loss(pred, yb, wb, args.huber_beta)
            loss.backward()
            optimizer.step()
            train_loss_sum += float(loss.item()) * xb.shape[0]
            train_count += xb.shape[0]

        val_metrics = evaluate(model, x_val_t, y_val_t)
        if val_metrics["mae_score"] < best_mae:
            best_mae = val_metrics["mae_score"]
            best_epoch = epoch
            best_state = {k: v.detach().cpu().clone() for k, v in model.state_dict().items()}

        if (epoch == 1) or (epoch % 20 == 0) or (epoch == args.epochs):
            train_loss = train_loss_sum / max(1, train_count)
            print(
                f"epoch {epoch:4d}/{args.epochs} "
                f"loss={train_loss:.6f} "
                f"val_mae={val_metrics['mae_score']:.3f} "
                f"val_rmse={val_metrics['rmse_score']:.3f}"
            )

    if best_state is not None:
        model.load_state_dict(best_state)

    final_metrics = evaluate(model, x_val_t, y_val_t)
    val_groups = [int(meta["sample_meta"][int(i)]["z_bin"]) for i in val_idx]
    group_metrics = evaluate_by_group(model, x_val_t, y_val_t, val_groups)
    distance_label_hist = build_distance_label_hist(y, meta["sample_meta"])
    train_distance_label_hist = build_distance_label_hist(y, meta["sample_meta"], train_idx)
    val_distance_label_hist = build_distance_label_hist(y, meta["sample_meta"], val_idx)

    args.out.mkdir(parents=True, exist_ok=True)
    onnx_path = args.out / "gesture_openness_cnn.onnx"
    norm_path = args.out / "cnn_norm_params.json"
    report_path = args.out / "cnn_train_report.json"
    checkpoint_path = args.out / "gesture_openness_cnn.pt"

    dummy = torch.zeros(1, CNN_CHANNELS, GRID_SIZE, GRID_SIZE, dtype=torch.float32)
    torch.onnx.export(
        model,
        dummy,
        onnx_path,
        input_names=["features"],
        output_names=["openness_norm"],
        dynamic_axes={"features": {0: "batch"}, "openness_norm": {0: "batch"}},
        opset_version=13,
    )

    torch.save(
        {
            "model_state_dict": model.state_dict(),
            "input_shape": [CNN_CHANNELS, GRID_SIZE, GRID_SIZE],
            "dense_hidden": int(args.dense_hidden),
            "huber_beta": float(args.huber_beta),
            "sample_weights": not args.no_sample_weights,
            "channel_mean": channel_mean,
            "channel_std": channel_std,
        },
        checkpoint_path,
    )

    norm_params = {
        "input_shape": [CNN_CHANNELS, GRID_SIZE, GRID_SIZE],
        "input_format": "NCHW",
        "label_scale": 100.0,
        "channel_names": meta["channel_names"],
        "channel_mean": channel_mean.tolist(),
        "channel_std": channel_std.tolist(),
        "preprocess": meta["preprocess"],
    }
    norm_path.write_text(json.dumps(norm_params, ensure_ascii=False, indent=2), encoding="utf-8")

    report = {
        "csv": str(args.csv),
        "train_rows": int(x_train.shape[0]),
        "val_rows": int(x_val.shape[0]),
        "epochs": int(args.epochs),
        "best_epoch": int(best_epoch),
        "dense_hidden": int(args.dense_hidden),
        "loss": {
            "type": "weighted_huber",
            "huber_beta": float(args.huber_beta),
            "sample_weights": not args.no_sample_weights,
            "weight_clip": float(args.weight_clip),
        },
        "metrics": final_metrics,
        "group_metrics": group_metrics,
        "split": split_meta,
        "preprocess": meta["preprocess"],
        "z_group_hist": meta["z_group_hist"],
        "label_hist": meta["label_hist"],
        "distance_label_hist": distance_label_hist,
        "train_distance_label_hist": train_distance_label_hist,
        "val_distance_label_hist": val_distance_label_hist,
        "dropped": meta["dropped"],
        "onnx": str(onnx_path),
        "norm_params": str(norm_path),
        "checkpoint": str(checkpoint_path),
    }
    report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")

    print("\nDone.")
    print(f"used rows: train={x_train.shape[0]}, val={x_val.shape[0]}")
    print(f"input shape: 1x{CNN_CHANNELS}x{GRID_SIZE}x{GRID_SIZE}")
    print(f"label hist: {meta['label_hist']}")
    print(f"distance x label hist: {distance_label_hist}")
    print(f"dropped: {meta['dropped']}")
    print(f"best epoch: {best_epoch}")
    print(f"val MAE:  {final_metrics['mae_score']:.3f} score")
    print(f"val RMSE: {final_metrics['rmse_score']:.3f} score")
    print(f"split: {split_meta}")
    print(f"z groups: {meta['z_group_hist']}")
    for name, metrics in group_metrics.items():
        print(f"val {name}: n={metrics['n']} MAE={metrics['mae_score']:.3f} RMSE={metrics['rmse_score']:.3f}")
    print(f"ONNX: {onnx_path}")
    print(f"norm: {norm_path}")
    print(f"report: {report_path}")


def main() -> None:
    args = parse_args()
    if args.eval_both_splits:
        base_out = args.out
        for split in ("random", "z-holdout"):
            split_args = argparse.Namespace(**vars(args))
            split_args.eval_both_splits = False
            split_args.val_split = split
            split_args.out = base_out / split
            print(f"\n=== train split: {split} ===")
            train_model(split_args)
        return

    train_model(args)


if __name__ == "__main__":
    main()
