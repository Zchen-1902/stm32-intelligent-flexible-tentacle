#!/usr/bin/env python3
"""Train a small MLP to regress hand openness from VL53 distance + CNH CSV data."""

from __future__ import annotations

import argparse
import csv
import json
import math
import random
from pathlib import Path
from typing import Iterable

import numpy as np
import torch
from torch import nn
from torch.utils.data import DataLoader, TensorDataset


LABEL_NONE = 255
ZONE_COUNT = 64
CNH_BINS = 8
VALID_TARGET_STATUS = (5, 9)
DISTANCE_MIN_MM = 80.0
FOREGROUND_MAX_MM = 900.0
FOREGROUND_MARGIN_MM = 180.0
REL_DISTANCE_SCALE_MM = 200.0
CNH_EPS = 1e-6
Z_BIN_EDGES_MM = [220.0, 280.0, 340.0, 420.0]
EXPECTED_BASE_COLUMNS = 5
EXPECTED_FEATURE_COLUMNS = ZONE_COUNT + ZONE_COUNT + ZONE_COUNT + (ZONE_COUNT * CNH_BINS)
EXPECTED_TOTAL_COLUMNS = EXPECTED_BASE_COLUMNS + EXPECTED_FEATURE_COLUMNS


class OpennessMLP(nn.Module):
    """小型回归网络：输入一帧VL53特征，输出0~1的张握程度。"""

    def __init__(self, input_dim: int, hidden: Iterable[int]) -> None:
        super().__init__()
        layers: list[nn.Module] = []
        last_dim = input_dim
        for hidden_dim in hidden:
            layers.append(nn.Linear(last_dim, hidden_dim))
            layers.append(nn.ReLU())
            last_dim = hidden_dim
        layers.append(nn.Linear(last_dim, 1))
        layers.append(nn.Sigmoid())
        self.net = nn.Sequential(*layers)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.net(x)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Train VL53 openness regression model.")
    parser.add_argument("--csv", required=True, type=Path, help="Path to gesture.csv.")
    parser.add_argument("--out", type=Path, default=Path("H743VIT6/Training/exported"), help="Output directory.")
    parser.add_argument("--epochs", type=int, default=300)
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--lr", type=float, default=1e-3)
    parser.add_argument("--val-ratio", type=float, default=0.2)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--hidden", type=int, nargs="+", default=[32, 16], help="Hidden layer sizes, e.g. --hidden 32 16.")
    parser.add_argument("--min-valid-zones", type=int, default=3, help="Drop rows with fewer valid zones.")
    parser.add_argument("--no-quality-filter", action="store_true", help="Do not filter by target_status/nb_target_detected.")
    parser.add_argument(
        "--val-split",
        choices=["random", "z-holdout"],
        default="random",
        help="random=普通随机验证；z-holdout=按手部距离段留出验证。",
    )
    return parser.parse_args()


def require_columns(header: list[str], columns: list[str]) -> None:
    missing = [name for name in columns if name not in header]
    if missing:
        raise ValueError(f"CSV missing required columns: {missing[:8]}{'...' if len(missing) > 8 else ''}")


def build_column_names() -> tuple[list[str], list[str], list[str], list[str]]:
    distance_cols = [f"d{i}" for i in range(ZONE_COUNT)]
    status_cols = [f"target_status_z{i}" for i in range(ZONE_COUNT)]
    nb_cols = [f"nb_target_z{i}" for i in range(ZONE_COUNT)]
    cnh_cols = [f"cnh_z{zone}_b{bin_idx}" for zone in range(ZONE_COUNT) for bin_idx in range(CNH_BINS)]
    return distance_cols, status_cols, nb_cols, cnh_cols


def build_feature_names() -> list[str]:
    distance_features = [f"rel_d{i}" for i in range(ZONE_COUNT)]
    cnh_features = [f"fg_cnh_z{zone}_b{bin_idx}" for zone in range(ZONE_COUNT) for bin_idx in range(CNH_BINS)]
    return distance_features + cnh_features


def z_group_name(group_id: int) -> str:
    if group_id <= 0:
        return f"<{int(Z_BIN_EDGES_MM[0])}mm"
    if group_id >= len(Z_BIN_EDGES_MM):
        return f">={int(Z_BIN_EDGES_MM[-1])}mm"
    return f"{int(Z_BIN_EDGES_MM[group_id - 1])}-{int(Z_BIN_EDGES_MM[group_id])}mm"


def preprocess_frame(
    distance: np.ndarray,
    target_status: np.ndarray,
    nb_target: np.ndarray,
    cnh: np.ndarray,
    min_valid_zones: int,
    quality_filter: bool,
) -> tuple[np.ndarray, dict] | None:
    """从单帧原始数据中提取前景手部特征。

    center_z 是前景 zone 距离中位数估算值，用于消除手离传感器远近带来的偏置。
    """

    valid_mask = (distance > DISTANCE_MIN_MM) & (distance < FOREGROUND_MAX_MM)
    if quality_filter:
        valid_mask &= np.isin(target_status, VALID_TARGET_STATUS) & (nb_target > 0)

    if int(np.sum(valid_mask)) < min_valid_zones:
        return None

    d_min = float(np.min(distance[valid_mask]))
    hand_mask = valid_mask & (distance <= d_min + FOREGROUND_MARGIN_MM)
    hand_count = int(np.sum(hand_mask))
    if hand_count < min_valid_zones:
        return None

    center_z = float(np.median(distance[hand_mask]))
    zones = np.flatnonzero(hand_mask)
    weights = 1.0 / np.maximum(distance[hand_mask], 1.0)
    weight_sum = float(np.sum(weights))
    center_x = float(np.sum((zones % 8) * weights) / weight_sum)
    center_y = float(np.sum((zones // 8) * weights) / weight_sum)

    rel_distance = np.zeros(ZONE_COUNT, dtype=np.float32)
    rel_distance[hand_mask] = np.clip(
        (distance[hand_mask] - center_z) / REL_DISTANCE_SCALE_MM,
        -3.0,
        3.0,
    )

    cnh_matrix = cnh.reshape(ZONE_COUNT, CNH_BINS).astype(np.float32)
    cnh_feature = np.zeros_like(cnh_matrix)
    cnh_feature[hand_mask, :] = cnh_matrix[hand_mask, :]
    cnh_scale = float(np.max(np.abs(cnh_feature[hand_mask, :])))
    if cnh_scale < CNH_EPS:
        cnh_scale = 1.0
    cnh_feature /= cnh_scale

    features = np.concatenate([rel_distance, cnh_feature.reshape(-1)]).astype(np.float32)
    z_bin = int(np.digitize(center_z, Z_BIN_EDGES_MM))
    frame_meta = {
        "center_x": center_x,
        "center_y": center_y,
        "center_z": center_z,
        "hand_zone_count": hand_count,
        "z_bin": z_bin,
    }
    return features, frame_meta


def read_dataset(csv_path: Path, min_valid_zones: int, quality_filter: bool) -> tuple[np.ndarray, np.ndarray, dict]:
    """读取CSV并生成训练矩阵。

    模型输入为前景相对距离[64] + 前景归一化CNH[512] 共576维。
    target_status/nb_target_detected 只用于清洗，不直接作为模型输入。
    """

    distance_cols, status_cols, nb_cols, cnh_cols = build_column_names()
    feature_cols = build_feature_names()
    base_cols = ["timestamp_ms", "label_score", "frame_count", "mode", "valid"]

    rows: list[list[float]] = []
    labels: list[float] = []
    label_hist: dict[str, int] = {}
    dropped = {
        "label_none": 0,
        "bad_label": 0,
        "invalid_frame": 0,
        "few_valid_zones": 0,
        "few_foreground_zones": 0,
        "parse_error": 0,
    }
    sample_meta: list[dict] = []
    z_group_hist: dict[str, int] = {}

    with csv_path.open("r", newline="", encoding="utf-8-sig") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            raise ValueError("CSV has no header.")

        header = reader.fieldnames
        require_columns(header, base_cols + distance_cols + status_cols + nb_cols + cnh_cols)
        if len(header) != EXPECTED_TOTAL_COLUMNS:
            print(f"[WARN] CSV header columns={len(header)}, expected={EXPECTED_TOTAL_COLUMNS}. Continue by column names.")

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
                processed = preprocess_frame(distance, target_status, nb_target, cnh, min_valid_zones, quality_filter)
                if processed is None:
                    dropped["few_foreground_zones"] += 1
                    continue
                features, frame_meta = processed
            except (TypeError, ValueError, KeyError):
                dropped["parse_error"] += 1
                continue

            rows.append(features)
            sample_meta.append(frame_meta)
            labels.append(label / 100.0)
            label_key = str(label)
            label_hist[label_key] = label_hist.get(label_key, 0) + 1
            z_key = z_group_name(int(frame_meta["z_bin"]))
            z_group_hist[z_key] = z_group_hist.get(z_key, 0) + 1

    if not rows:
        raise ValueError("No usable training rows after filtering.")

    x = np.asarray(rows, dtype=np.float32)
    y = np.asarray(labels, dtype=np.float32).reshape(-1, 1)
    meta = {
        "feature_columns": feature_cols,
        "preprocess": {
            "mode": "foreground_relative",
            "distance_min_mm": DISTANCE_MIN_MM,
            "foreground_max_mm": FOREGROUND_MAX_MM,
            "foreground_margin_mm": FOREGROUND_MARGIN_MM,
            "relative_distance_scale_mm": REL_DISTANCE_SCALE_MM,
            "cnh_normalization": "per_frame_foreground_abs_max",
        },
        "sample_meta": sample_meta,
        "z_group_hist": dict(sorted(z_group_hist.items())),
        "label_hist": dict(sorted(label_hist.items(), key=lambda item: int(item[0]))),
        "dropped": dropped,
        "total_used": int(x.shape[0]),
        "input_dim": int(x.shape[1]),
    }
    return x, y, meta


def split_indices(
    row_count: int,
    val_ratio: float,
    seed: int,
    sample_meta: list[dict],
    mode: str,
) -> tuple[np.ndarray, np.ndarray, dict]:
    if not 0.0 < val_ratio < 0.5:
        raise ValueError("--val-ratio should be between 0 and 0.5")

    rng = np.random.default_rng(seed)
    indices = np.arange(row_count)
    rng.shuffle(indices)
    val_count = max(1, int(round(row_count * val_ratio)))
    val_idx = indices[:val_count]
    train_idx = indices[val_count:]
    if train_idx.size == 0:
        raise ValueError("Not enough rows for train/val split.")

    if mode == "random":
        return train_idx, val_idx, {"mode": "random"}

    groups = np.asarray([int(item["z_bin"]) for item in sample_meta], dtype=np.int16)
    counts = {int(group): int(np.sum(groups == group)) for group in sorted(set(groups.tolist()))}
    candidates = [group for group, count in counts.items() if 0 < count < row_count]
    if not candidates:
        return train_idx, val_idx, {"mode": "random", "fallback_reason": "no_valid_z_group"}

    target_count = max(1, int(round(row_count * val_ratio)))
    val_group = min(candidates, key=lambda group: abs(counts[group] - target_count))
    val_idx = np.flatnonzero(groups == val_group)
    train_idx = np.flatnonzero(groups != val_group)
    if train_idx.size == 0:
        return indices[val_count:], indices[:val_count], {"mode": "random", "fallback_reason": "empty_train_after_z_holdout"}

    return train_idx, val_idx, {
        "mode": "z-holdout",
        "val_z_group": z_group_name(int(val_group)),
        "z_group_counts": {z_group_name(group): count for group, count in counts.items()},
    }


def normalize_train_val(x_train: np.ndarray, x_val: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    mean = x_train.mean(axis=0)
    std = x_train.std(axis=0)
    std = np.where(std < 1e-6, 1.0, std)
    return (x_train - mean) / std, (x_val - mean) / std, mean.astype(np.float32), std.astype(np.float32)


def evaluate(model: nn.Module, x: torch.Tensor, y: torch.Tensor) -> dict:
    model.eval()
    with torch.no_grad():
        pred = model(x)
        err_score = (pred - y) * 100.0
        mae = torch.mean(torch.abs(err_score)).item()
        rmse = torch.sqrt(torch.mean(err_score * err_score)).item()
        max_abs = torch.max(torch.abs(err_score)).item()
    return {"mae_score": mae, "rmse_score": rmse, "max_abs_score": max_abs}


def evaluate_by_group(model: nn.Module, x: torch.Tensor, y: torch.Tensor, groups: list[int]) -> dict:
    result = {}
    for group in sorted(set(groups)):
        idx = [i for i, item in enumerate(groups) if item == group]
        metrics = evaluate(model, x[idx], y[idx])
        metrics["n"] = len(idx)
        result[z_group_name(int(group))] = metrics
    return result


def train_model(args: argparse.Namespace) -> None:
    random.seed(args.seed)
    np.random.seed(args.seed)
    torch.manual_seed(args.seed)

    x, y, meta = read_dataset(args.csv, args.min_valid_zones, quality_filter=not args.no_quality_filter)
    train_idx, val_idx, split_meta = split_indices(x.shape[0], args.val_ratio, args.seed, meta["sample_meta"], args.val_split)
    x_train, y_train = x[train_idx], y[train_idx]
    x_val, y_val = x[val_idx], y[val_idx]
    x_train_n, x_val_n, mean, std = normalize_train_val(x_train, x_val)

    train_ds = TensorDataset(torch.from_numpy(x_train_n), torch.from_numpy(y_train))
    train_loader = DataLoader(train_ds, batch_size=args.batch_size, shuffle=True)
    x_val_t = torch.from_numpy(x_val_n)
    y_val_t = torch.from_numpy(y_val)

    model = OpennessMLP(input_dim=x.shape[1], hidden=args.hidden)
    optimizer = torch.optim.Adam(model.parameters(), lr=args.lr)
    loss_fn = nn.MSELoss()

    best_state = None
    best_mae = math.inf
    best_epoch = 0

    for epoch in range(1, args.epochs + 1):
        model.train()
        train_loss_sum = 0.0
        train_count = 0

        for xb, yb in train_loader:
            optimizer.zero_grad(set_to_none=True)
            pred = model(xb)
            loss = loss_fn(pred, yb)
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

    args.out.mkdir(parents=True, exist_ok=True)
    onnx_path = args.out / "gesture_openness.onnx"
    norm_path = args.out / "norm_params.json"
    report_path = args.out / "train_report.json"
    checkpoint_path = args.out / "gesture_openness.pt"

    dummy = torch.zeros(1, x.shape[1], dtype=torch.float32)
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
            "input_dim": int(x.shape[1]),
            "hidden": list(args.hidden),
            "mean": mean,
            "std": std,
        },
        checkpoint_path,
    )

    norm_params = {
        "input_dim": int(x.shape[1]),
        "label_scale": 100.0,
        "feature_columns": meta["feature_columns"],
        "mean": mean.tolist(),
        "std": std.tolist(),
    }
    norm_path.write_text(json.dumps(norm_params, ensure_ascii=False, indent=2), encoding="utf-8")

    report = {
        "csv": str(args.csv),
        "train_rows": int(x_train.shape[0]),
        "val_rows": int(x_val.shape[0]),
        "hidden": list(args.hidden),
        "epochs": int(args.epochs),
        "best_epoch": int(best_epoch),
        "metrics": final_metrics,
        "group_metrics": group_metrics,
        "split": split_meta,
        "preprocess": meta["preprocess"],
        "z_group_hist": meta["z_group_hist"],
        "label_hist": meta["label_hist"],
        "dropped": meta["dropped"],
        "onnx": str(onnx_path),
        "norm_params": str(norm_path),
        "checkpoint": str(checkpoint_path),
    }
    report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")

    print("\nDone.")
    print(f"used rows: train={x_train.shape[0]}, val={x_val.shape[0]}")
    print(f"label hist: {meta['label_hist']}")
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
    train_model(args)


if __name__ == "__main__":
    main()
