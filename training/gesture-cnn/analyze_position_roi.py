#!/usr/bin/env python3
"""离线比较前景中心和 ROI 中心的位置估计。

用途：
- 不改 H7 固件；
- 用同一帧 CSV 数据同时计算普通前景中心和 ROI 内前景中心；
- 导出逐帧对比 CSV，辅助判断后续控制方向应该用哪种位置估计。
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Any

import numpy as np

ZONE_COUNT = 64
GRID_SIZE = 8
VALID_TARGET_STATUS = (5, 9)
LABEL_NONE = 255

DISTANCE_MIN_MM = 160.0
DISTANCE_MAX_MM = 450.0
FOREGROUND_MARGIN_MM = 120.0
Z_BIN_EDGES_MM = [220.0, 280.0, 340.0, 420.0]

ROI_PALM_WIDTH_MM = 150.0
ROI_PALM_HEIGHT_MM = 170.0
ROI_TAN_HALF_FOV = 0.41421356  # tan(22.5deg)，VL53L8CH 约 45deg x 45deg 方形 FoV。
ROI_MIN_SIZE = 2
ROI_ROWS_FOR_Z_REF = 3


COMPARE_COLUMNS = [
    "row_index",
    "timestamp_ms",
    "frame_count",
    "label_score",
    "valid_frame",
    "drop_reason",
    "valid_zone_count",
    "d_min_mm",
    "fg_valid",
    "fg_count",
    "fg_x",
    "fg_y",
    "fg_z_mm",
    "fg_x_ctrl",
    "fg_y_ctrl",
    "fg_bbox_x_min",
    "fg_bbox_x_max",
    "fg_bbox_y_min",
    "fg_bbox_y_max",
    "fg_bbox_width",
    "fg_bbox_height",
    "roi_valid",
    "roi_count",
    "roi_x",
    "roi_y",
    "roi_z_mm",
    "roi_x_ctrl",
    "roi_y_ctrl",
    "roi_x_min",
    "roi_x_max",
    "roi_y_min",
    "roi_y_max",
    "roi_width",
    "roi_height",
    "roi_tip_x",
    "roi_tip_y",
    "roi_z_ref_mm",
    "dx_roi_minus_fg",
    "dy_roi_minus_fg",
    "dz_roi_minus_fg_mm",
    "z_group",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Compare foreground center and ROI center from VL53 gesture CSV.")
    parser.add_argument("--csv", required=True, type=Path, help="输入 gesture.csv 或合并后的训练 CSV。")
    parser.add_argument("--out", type=Path, default=None, help="逐帧对比 CSV 输出路径。")
    parser.add_argument("--summary", type=Path, default=None, help="统计摘要 JSON 输出路径。")
    parser.add_argument("--plot-dir", type=Path, default=None, help="可选：输出少量热力图 PNG 的目录。")
    parser.add_argument("--plot-count", type=int, default=0, help="可选：生成前 N 帧有效样本热力图。")
    parser.add_argument("--plot-step", type=int, default=1, help="热力图抽样步长，1表示连续取。")
    parser.add_argument("--min-valid-zones", type=int, default=3, help="前景或 ROI 至少需要的有效 zone 数。")
    parser.add_argument("--no-quality-filter", action="store_true", help="不使用 target_status/nb_target 过滤。")
    parser.add_argument("--roi-tip", choices=["top", "bottom"], default="top", help="手掌前端在 8x8 图中的方向。")
    return parser.parse_args()


def build_column_names() -> tuple[list[str], list[str], list[str]]:
    distance_cols = [f"d{i}" for i in range(ZONE_COUNT)]
    status_cols = [f"target_status_z{i}" for i in range(ZONE_COUNT)]
    nb_cols = [f"nb_target_z{i}" for i in range(ZONE_COUNT)]
    return distance_cols, status_cols, nb_cols


def require_columns(header: list[str], columns: list[str]) -> None:
    missing = [name for name in columns if name not in header]
    if missing:
        raise ValueError(f"CSV缺少必要列: {missing[:8]}{'...' if len(missing) > 8 else ''}")


def z_group_name(z_mm: float) -> str:
    group_id = int(np.digitize(z_mm, Z_BIN_EDGES_MM))
    if group_id <= 0:
        return f"<{int(Z_BIN_EDGES_MM[0])}mm"
    if group_id >= len(Z_BIN_EDGES_MM):
        return f">={int(Z_BIN_EDGES_MM[-1])}mm"
    return f"{int(Z_BIN_EDGES_MM[group_id - 1])}-{int(Z_BIN_EDGES_MM[group_id])}mm"


def clip_control(value: float) -> int:
    value = max(-100.0, min(100.0, value))
    return int(value + 0.5) if value >= 0.0 else int(value - 0.5)


def x_to_control(x: float) -> int:
    return clip_control((x - 3.5) * 100.0 / 3.5)


def y_to_control(y: float) -> int:
    return clip_control((y - 3.5) * 100.0 / 3.5)


def calc_center(mask: np.ndarray, distance: np.ndarray) -> dict[str, Any] | None:
    """计算一个 mask 内的加权中心和外接框。"""
    zones = np.flatnonzero(mask)
    if zones.size == 0:
        return None

    xs = zones % GRID_SIZE
    ys = zones // GRID_SIZE
    dist = distance[mask]
    weights = 1.0 / np.maximum(dist, 1.0)
    weight_sum = float(np.sum(weights))
    if not weight_sum > 0.0:
        return None

    center_x = float(np.sum(xs * weights) / weight_sum)
    center_y = float(np.sum(ys * weights) / weight_sum)
    center_z = float(np.median(dist))
    return {
        "count": int(zones.size),
        "x": center_x,
        "y": center_y,
        "z_mm": center_z,
        "x_ctrl": x_to_control(center_x),
        "y_ctrl": y_to_control(center_y),
        "bbox_x_min": int(np.min(xs)),
        "bbox_x_max": int(np.max(xs)),
        "bbox_y_min": int(np.min(ys)),
        "bbox_y_max": int(np.max(ys)),
        "bbox_width": int(np.max(xs) - np.min(xs) + 1),
        "bbox_height": int(np.max(ys) - np.min(ys) + 1),
    }


def clip_roi_cells(value: int) -> int:
    return max(ROI_MIN_SIZE, min(GRID_SIZE, int(value)))


def calc_roi_cells(size_mm: float, z_ref_mm: float) -> int:
    """按距离估算手掌宽高大约覆盖多少个 8x8 zone。"""
    if z_ref_mm <= 1.0:
        return GRID_SIZE
    cell_mm = 2.0 * float(z_ref_mm) * ROI_TAN_HALF_FOV / float(GRID_SIZE)
    if cell_mm <= 1.0:
        return GRID_SIZE
    return clip_roi_cells(math.ceil(float(size_mm) / cell_mm))


def centered_range(center: int, width: int) -> tuple[int, int]:
    width = clip_roi_cells(width)
    start = int(center) - (width // 2)
    end = start + width - 1
    if start < 0:
        end -= start
        start = 0
    if end >= GRID_SIZE:
        start -= end - (GRID_SIZE - 1)
        end = GRID_SIZE - 1
    start = max(0, start)
    return start, end


def build_roi_mask(raw_foreground: np.ndarray, distance: np.ndarray, tip_is_top: bool) -> tuple[np.ndarray, dict[str, Any]]:
    """根据原始前景估算 ROI 矩形。"""
    roi_mask = np.zeros(ZONE_COUNT, dtype=bool)
    zones = np.flatnonzero(raw_foreground)
    if zones.size == 0:
        return roi_mask, {"roi_valid": False, "reason": "roi_no_foreground"}

    xs = zones % GRID_SIZE
    ys = zones // GRID_SIZE
    tip_y = int(np.min(ys) if tip_is_top else np.max(ys))
    tip_row_xs = xs[ys == tip_y]
    tip_x = int((int(np.min(tip_row_xs)) + int(np.max(tip_row_xs))) // 2)

    distance_grid = distance.reshape(GRID_SIZE, GRID_SIZE)
    fg_grid = raw_foreground.reshape(GRID_SIZE, GRID_SIZE)
    row_step = 1 if tip_is_top else -1

    z_samples: list[float] = []
    for offset in range(ROI_ROWS_FOR_Z_REF):
        yy = tip_y + offset * row_step
        if yy < 0 or yy >= GRID_SIZE:
            continue
        row_mask = fg_grid[yy]
        if np.any(row_mask):
            z_samples.extend(distance_grid[yy, row_mask].astype(np.float32).tolist())

    if not z_samples:
        return roi_mask, {"roi_valid": False, "reason": "roi_no_z_ref"}

    z_ref_mm = float(np.median(np.asarray(z_samples, dtype=np.float32)))
    roi_w = calc_roi_cells(ROI_PALM_WIDTH_MM, z_ref_mm)
    roi_h = calc_roi_cells(ROI_PALM_HEIGHT_MM, z_ref_mm)
    x_min, x_max = centered_range(tip_x, roi_w)

    if tip_is_top:
        y_min = tip_y
        y_max = min(GRID_SIZE - 1, tip_y + roi_h - 1)
    else:
        y_max = tip_y
        y_min = max(0, tip_y - roi_h + 1)

    roi_grid = roi_mask.reshape(GRID_SIZE, GRID_SIZE)
    roi_grid[y_min : y_max + 1, x_min : x_max + 1] = True

    return roi_mask, {
        "roi_valid": True,
        "roi_x_min": x_min,
        "roi_x_max": x_max,
        "roi_y_min": y_min,
        "roi_y_max": y_max,
        "roi_width": int(x_max - x_min + 1),
        "roi_height": int(y_max - y_min + 1),
        "roi_tip_x": tip_x,
        "roi_tip_y": tip_y,
        "roi_z_ref_mm": z_ref_mm,
    }


def empty_output(row_index: int, row: dict[str, str], reason: str) -> dict[str, Any]:
    return {
        "row_index": row_index,
        "timestamp_ms": row.get("timestamp_ms", ""),
        "frame_count": row.get("frame_count", ""),
        "label_score": row.get("label_score", ""),
        "valid_frame": row.get("valid", ""),
        "drop_reason": reason,
    }


def analyze_row(
    row_index: int,
    row: dict[str, str],
    distance_cols: list[str],
    status_cols: list[str],
    nb_cols: list[str],
    *,
    min_valid_zones: int,
    quality_filter: bool,
    tip_is_top: bool,
) -> tuple[dict[str, Any], dict[str, Any] | None]:
    try:
        label = int(row["label_score"])
        valid_frame = int(row["valid"])
        distance = np.asarray([float(row[name]) for name in distance_cols], dtype=np.float32)
        target_status = np.asarray([int(row[name]) for name in status_cols], dtype=np.int16)
        nb_target = np.asarray([int(row[name]) for name in nb_cols], dtype=np.int16)
    except (KeyError, TypeError, ValueError):
        return empty_output(row_index, row, "parse_error"), None

    if label == LABEL_NONE:
        return empty_output(row_index, row, "label_none"), None
    if valid_frame != 1:
        return empty_output(row_index, row, "invalid_frame"), None

    status_ok = np.isin(target_status, VALID_TARGET_STATUS)
    valid_mask = np.isfinite(distance) & (distance >= DISTANCE_MIN_MM) & (distance <= DISTANCE_MAX_MM)
    if quality_filter:
        valid_mask &= status_ok & (nb_target > 0)

    valid_count = int(np.sum(valid_mask))
    if valid_count < min_valid_zones:
        out = empty_output(row_index, row, "few_valid_zones")
        out["valid_zone_count"] = valid_count
        return out, None

    d_min = float(np.min(distance[valid_mask]))
    foreground_mask = valid_mask & (distance <= d_min + FOREGROUND_MARGIN_MM)
    fg_count = int(np.sum(foreground_mask))
    if fg_count < min_valid_zones:
        out = empty_output(row_index, row, "few_foreground_zones")
        out["valid_zone_count"] = valid_count
        out["d_min_mm"] = d_min
        return out, None

    fg = calc_center(foreground_mask, distance)
    if fg is None:
        return empty_output(row_index, row, "foreground_center_fail"), None

    roi_mask, roi_meta = build_roi_mask(foreground_mask, distance, tip_is_top)
    roi_foreground = foreground_mask & roi_mask
    roi = calc_center(roi_foreground, distance) if int(np.sum(roi_foreground)) >= min_valid_zones else None

    out: dict[str, Any] = {
        "row_index": row_index,
        "timestamp_ms": row.get("timestamp_ms", ""),
        "frame_count": row.get("frame_count", ""),
        "label_score": label,
        "valid_frame": valid_frame,
        "drop_reason": "",
        "valid_zone_count": valid_count,
        "d_min_mm": round(d_min, 3),
        "fg_valid": 1,
        "fg_count": fg["count"],
        "fg_x": round(fg["x"], 4),
        "fg_y": round(fg["y"], 4),
        "fg_z_mm": round(fg["z_mm"], 3),
        "fg_x_ctrl": fg["x_ctrl"],
        "fg_y_ctrl": fg["y_ctrl"],
        "fg_bbox_x_min": fg["bbox_x_min"],
        "fg_bbox_x_max": fg["bbox_x_max"],
        "fg_bbox_y_min": fg["bbox_y_min"],
        "fg_bbox_y_max": fg["bbox_y_max"],
        "fg_bbox_width": fg["bbox_width"],
        "fg_bbox_height": fg["bbox_height"],
        "roi_valid": 0,
        "z_group": z_group_name(float(fg["z_mm"])),
    }

    for key in ["roi_x_min", "roi_x_max", "roi_y_min", "roi_y_max", "roi_width", "roi_height", "roi_tip_x", "roi_tip_y", "roi_z_ref_mm"]:
        out[key] = roi_meta.get(key, "")

    plot_payload = {
        "row_index": row_index,
        "label": label,
        "distance": distance.copy(),
        "foreground_mask": foreground_mask.copy(),
        "roi_mask": roi_mask.copy(),
        "fg": fg,
        "roi": roi,
        "roi_meta": roi_meta,
    }

    if roi is not None:
        out.update(
            {
                "roi_valid": 1,
                "roi_count": roi["count"],
                "roi_x": round(roi["x"], 4),
                "roi_y": round(roi["y"], 4),
                "roi_z_mm": round(roi["z_mm"], 3),
                "roi_x_ctrl": roi["x_ctrl"],
                "roi_y_ctrl": roi["y_ctrl"],
                "dx_roi_minus_fg": round(float(roi["x"] - fg["x"]), 4),
                "dy_roi_minus_fg": round(float(roi["y"] - fg["y"]), 4),
                "dz_roi_minus_fg_mm": round(float(roi["z_mm"] - fg["z_mm"]), 3),
            }
        )
    else:
        out["drop_reason"] = "roi_few_foreground_zones"

    return out, plot_payload


def update_summary(summary: dict[str, Any], out: dict[str, Any]) -> None:
    reason = str(out.get("drop_reason", ""))
    if reason:
        summary["drop_reasons"][reason] = summary["drop_reasons"].get(reason, 0) + 1

    if out.get("fg_valid") == 1:
        summary["fg_valid_rows"] += 1
        label_key = str(out.get("label_score", ""))
        z_key = str(out.get("z_group", ""))
        summary["by_label"][label_key] = summary["by_label"].get(label_key, 0) + 1
        summary["by_z_group"][z_key] = summary["by_z_group"].get(z_key, 0) + 1

    if out.get("roi_valid") == 1:
        summary["roi_valid_rows"] += 1
        summary["abs_dx"].append(abs(float(out["dx_roi_minus_fg"])))
        summary["abs_dy"].append(abs(float(out["dy_roi_minus_fg"])))
        summary["abs_dz_mm"].append(abs(float(out["dz_roi_minus_fg_mm"])))
        if abs(float(out["dx_roi_minus_fg"])) > 0.75 or abs(float(out["dy_roi_minus_fg"])) > 0.75:
            summary["large_xy_diff_rows"] += 1


def finalize_summary(summary: dict[str, Any]) -> dict[str, Any]:
    result = {k: v for k, v in summary.items() if k not in ("abs_dx", "abs_dy", "abs_dz_mm")}
    for key in ("abs_dx", "abs_dy", "abs_dz_mm"):
        values = np.asarray(summary[key], dtype=np.float32)
        if values.size == 0:
            result[key] = {"mean": None, "median": None, "max": None}
        else:
            result[key] = {
                "mean": float(np.mean(values)),
                "median": float(np.median(values)),
                "max": float(np.max(values)),
            }
    return result


def plot_payloads(payloads: list[dict[str, Any]], plot_dir: Path) -> None:
    try:
        import matplotlib.pyplot as plt
        from matplotlib.patches import Rectangle
    except ImportError:
        print("[WARN] matplotlib 不可用，跳过热力图输出。")
        return

    plot_dir.mkdir(parents=True, exist_ok=True)
    for item in payloads:
        distance_grid = item["distance"].reshape(GRID_SIZE, GRID_SIZE)
        fg = item["fg"]
        roi = item["roi"]
        roi_meta = item["roi_meta"]

        fig, ax = plt.subplots(figsize=(5.2, 4.6))
        image = ax.imshow(distance_grid, cmap="viridis", origin="upper")
        fig.colorbar(image, ax=ax, label="distance mm")
        ax.scatter([fg["x"]], [fg["y"]], c="red", marker="x", s=90, label="foreground center")
        if roi is not None:
            ax.scatter([roi["x"]], [roi["y"]], c="white", edgecolors="blue", marker="o", s=80, label="ROI center")
        if roi_meta.get("roi_valid"):
            rect = Rectangle(
                (roi_meta["roi_x_min"] - 0.5, roi_meta["roi_y_min"] - 0.5),
                roi_meta["roi_width"],
                roi_meta["roi_height"],
                fill=False,
                edgecolor="white",
                linewidth=1.8,
            )
            ax.add_patch(rect)
        ax.set_xticks(range(GRID_SIZE))
        ax.set_yticks(range(GRID_SIZE))
        ax.set_title(f"row={item['row_index']} label={item['label']}")
        ax.legend(loc="upper right", fontsize=8)
        fig.tight_layout()
        fig.savefig(plot_dir / f"frame_{int(item['row_index']):05d}.png", dpi=150)
        plt.close(fig)


def main() -> None:
    args = parse_args()
    csv_path = args.csv
    out_path = args.out or csv_path.with_name(csv_path.stem + "_position_compare.csv")
    summary_path = args.summary or out_path.with_suffix(".summary.json")
    quality_filter = not args.no_quality_filter
    tip_is_top = args.roi_tip == "top"

    distance_cols, status_cols, nb_cols = build_column_names()
    base_cols = ["timestamp_ms", "label_score", "frame_count", "mode", "valid"]

    out_path.parent.mkdir(parents=True, exist_ok=True)
    summary_path.parent.mkdir(parents=True, exist_ok=True)

    summary: dict[str, Any] = {
        "csv": str(csv_path),
        "distance_min_mm": DISTANCE_MIN_MM,
        "distance_max_mm": DISTANCE_MAX_MM,
        "foreground_margin_mm": FOREGROUND_MARGIN_MM,
        "roi_palm_width_mm": ROI_PALM_WIDTH_MM,
        "roi_palm_height_mm": ROI_PALM_HEIGHT_MM,
        "roi_tip": args.roi_tip,
        "min_valid_zones": int(args.min_valid_zones),
        "quality_filter": quality_filter,
        "total_rows": 0,
        "fg_valid_rows": 0,
        "roi_valid_rows": 0,
        "large_xy_diff_rows": 0,
        "drop_reasons": {},
        "by_label": {},
        "by_z_group": {},
        "abs_dx": [],
        "abs_dy": [],
        "abs_dz_mm": [],
    }
    plot_payload_list: list[dict[str, Any]] = []
    plot_seen = 0

    with csv_path.open("r", newline="", encoding="utf-8-sig") as src, out_path.open("w", newline="", encoding="utf-8") as dst:
        reader = csv.DictReader(src)
        if reader.fieldnames is None:
            raise ValueError("CSV没有表头。")
        require_columns(reader.fieldnames, base_cols + distance_cols + status_cols + nb_cols)

        writer = csv.DictWriter(dst, fieldnames=COMPARE_COLUMNS, extrasaction="ignore")
        writer.writeheader()

        for row_index, row in enumerate(reader, start=1):
            summary["total_rows"] += 1
            out, payload = analyze_row(
                row_index,
                row,
                distance_cols,
                status_cols,
                nb_cols,
                min_valid_zones=args.min_valid_zones,
                quality_filter=quality_filter,
                tip_is_top=tip_is_top,
            )
            writer.writerow(out)
            update_summary(summary, out)

            if payload is not None and args.plot_count > 0:
                if plot_seen % max(1, args.plot_step) == 0 and len(plot_payload_list) < args.plot_count:
                    plot_payload_list.append(payload)
                plot_seen += 1

    final_summary = finalize_summary(summary)
    summary_path.write_text(json.dumps(final_summary, ensure_ascii=False, indent=2), encoding="utf-8")

    if args.plot_dir is not None and plot_payload_list:
        plot_payloads(plot_payload_list, args.plot_dir)

    print("Done.")
    print(f"compare_csv: {out_path}")
    print(f"summary_json: {summary_path}")
    print(f"total={final_summary['total_rows']} fg_valid={final_summary['fg_valid_rows']} roi_valid={final_summary['roi_valid_rows']}")
    print(f"abs_dx={final_summary['abs_dx']} abs_dy={final_summary['abs_dy']} abs_dz_mm={final_summary['abs_dz_mm']}")


if __name__ == "__main__":
    main()
