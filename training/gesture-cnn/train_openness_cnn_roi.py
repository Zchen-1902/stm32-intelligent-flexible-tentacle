"""ROI 前景版本 CNN 训练入口。

这个脚本不复制原来的 CNN 训练流程，只替换单帧前处理：
- 先用原来的有效距离和 d_min + margin 找原始前景；
- 再用手掌近似尺寸和 VL53 视场角估算一个 ROI 矩形；
- 最终只把 ROI 内的前景送入 20 通道 CNN 特征。

这样可以和 train_openness_cnn.py 使用同一套网络、训练参数、数据集做公平对比。
"""
from __future__ import annotations

import math
from typing import Tuple

import numpy as np

import train_openness_cnn as base
from train_openness import CNH_BINS, VALID_TARGET_STATUS, ZONE_COUNT

# ROI 只用于训练前处理，不改变模型结构，仍然输出 20x8x8。
ROI_TIP_IS_TOP = True
ROI_PALM_WIDTH_MM = 150.0
ROI_PALM_HEIGHT_MM = 170.0
ROI_TAN_HALF_FOV = 0.41421356  # tan(22.5deg)，VL53L8CH 约 45deg x 45deg 方形 FoV。
ROI_MIN_SIZE = 2
ROI_ROWS_FOR_Z_REF = 3


def _clip_cell_count(value: int) -> int:
    """限制 ROI 宽高格数，避免退化为 0 或超过 8x8。"""
    return max(ROI_MIN_SIZE, min(base.GRID_SIZE, int(value)))


def _calc_roi_cells(size_mm: float, z_ref_mm: float) -> int:
    """根据距离估算手掌尺寸大约覆盖多少个 zone。"""
    if z_ref_mm <= 1.0:
        return base.GRID_SIZE
    cell_mm = 2.0 * float(z_ref_mm) * ROI_TAN_HALF_FOV / float(base.GRID_SIZE)
    if cell_mm <= 1.0:
        return base.GRID_SIZE
    return _clip_cell_count(math.ceil(float(size_mm) / cell_mm))


def _centered_range(center: int, width: int) -> Tuple[int, int]:
    """生成以 center 为中心、限制在 0~7 内的闭区间。"""
    width = _clip_cell_count(width)
    start = int(center) - (width // 2)
    end = start + width - 1
    if start < 0:
        end -= start
        start = 0
    if end >= base.GRID_SIZE:
        start -= end - (base.GRID_SIZE - 1)
        end = base.GRID_SIZE - 1
    start = max(0, start)
    return start, end


def _build_roi_mask(raw_foreground: np.ndarray, distance: np.ndarray) -> Tuple[np.ndarray, dict]:
    """从原始前景估算手部 ROI 矩形，返回 64 点 bool mask 和调试信息。"""
    roi_mask = np.zeros(ZONE_COUNT, dtype=bool)
    fg_zones = np.flatnonzero(raw_foreground)
    if fg_zones.size == 0:
        return roi_mask, {"roi_valid": False}

    xs = fg_zones % base.GRID_SIZE
    ys = fg_zones // base.GRID_SIZE
    tip_y = int(np.min(ys) if ROI_TIP_IS_TOP else np.max(ys))
    tip_row_xs = xs[ys == tip_y]
    tip_x = int((int(np.min(tip_row_xs)) + int(np.max(tip_row_xs))) // 2)

    distance_grid = distance.reshape(base.GRID_SIZE, base.GRID_SIZE)
    fg_grid = raw_foreground.reshape(base.GRID_SIZE, base.GRID_SIZE)
    row_step = 1 if ROI_TIP_IS_TOP else -1

    z_samples = []
    for offset in range(ROI_ROWS_FOR_Z_REF):
        yy = tip_y + offset * row_step
        if yy < 0 or yy >= base.GRID_SIZE:
            continue
        row_mask = fg_grid[yy]
        if np.any(row_mask):
            z_samples.extend(distance_grid[yy, row_mask].astype(np.float32).tolist())

    if not z_samples:
        return roi_mask, {"roi_valid": False}

    z_ref_mm = float(np.median(np.asarray(z_samples, dtype=np.float32)))
    roi_w = _calc_roi_cells(ROI_PALM_WIDTH_MM, z_ref_mm)
    roi_h = _calc_roi_cells(ROI_PALM_HEIGHT_MM, z_ref_mm)

    x_min, x_max = _centered_range(tip_x, roi_w)
    if ROI_TIP_IS_TOP:
        y_min = tip_y
        y_max = min(base.GRID_SIZE - 1, tip_y + roi_h - 1)
    else:
        y_max = tip_y
        y_min = max(0, tip_y - roi_h + 1)

    roi_grid = roi_mask.reshape(base.GRID_SIZE, base.GRID_SIZE)
    roi_grid[y_min : y_max + 1, x_min : x_max + 1] = True

    return roi_mask, {
        "roi_valid": True,
        "roi_x_min": int(x_min),
        "roi_x_max": int(x_max),
        "roi_y_min": int(y_min),
        "roi_y_max": int(y_max),
        "roi_width": int(x_max - x_min + 1),
        "roi_height": int(y_max - y_min + 1),
        "roi_tip_x": int(tip_x),
        "roi_tip_y": int(tip_y),
        "roi_z_ref_mm": z_ref_mm,
        "raw_foreground_count": int(fg_zones.size),
    }


def preprocess_frame_cnn_roi(
    distance: np.ndarray,
    target_status: np.ndarray,
    nb_target: np.ndarray,
    cnh: np.ndarray,
    min_valid_zones: int,
    quality_filter: bool,
) -> tuple[np.ndarray, dict, str | None]:
    """把一帧 CSV 数据转成 ROI 前景版固定 20x8x8 CNN 输入。"""
    valid_mask = np.isfinite(distance) & (distance >= base.DISTANCE_MIN_MM) & (distance <= base.DISTANCE_MAX_MM)
    status_ok = np.isin(target_status, VALID_TARGET_STATUS)
    nb_norm = np.clip(nb_target.astype(np.float32), 0.0, 3.0) / 3.0
    if quality_filter:
        valid_mask &= status_ok & (nb_target > 0)

    valid_count = int(np.sum(valid_mask))
    if valid_count < min_valid_zones:
        return np.empty((0,), dtype=np.float32), {}, "few_valid_zones"

    d_min = float(np.min(distance[valid_mask]))
    raw_foreground = valid_mask & (distance <= d_min + base.FOREGROUND_MARGIN_MM)

    roi_mask, roi_meta = _build_roi_mask(raw_foreground, distance)
    if not roi_meta.get("roi_valid", False):
        return np.empty((0,), dtype=np.float32), {}, "few_foreground_zones"

    foreground_mask = raw_foreground & roi_mask
    foreground_count = int(np.sum(foreground_mask))
    if foreground_count < min_valid_zones:
        return np.empty((0,), dtype=np.float32), roi_meta, "few_foreground_zones"

    zones = np.flatnonzero(foreground_mask)
    xs = zones % base.GRID_SIZE
    ys = zones // base.GRID_SIZE
    center_z = float(np.median(distance[foreground_mask]))
    weights = 1.0 / np.maximum(distance[foreground_mask], 1.0)
    weight_sum = float(np.sum(weights))
    center_x = float(np.sum(xs * weights) / weight_sum)
    center_y = float(np.sum(ys * weights) / weight_sum)
    bbox_width = int(np.max(xs) - np.min(xs) + 1)
    bbox_height = int(np.max(ys) - np.min(ys) + 1)

    feature = np.zeros((base.CNN_CHANNELS, base.GRID_SIZE, base.GRID_SIZE), dtype=np.float32)
    dist_grid = distance.reshape(base.GRID_SIZE, base.GRID_SIZE)
    valid_grid = valid_mask.reshape(base.GRID_SIZE, base.GRID_SIZE)
    fg_grid = foreground_mask.reshape(base.GRID_SIZE, base.GRID_SIZE)

    rel = np.zeros((base.GRID_SIZE, base.GRID_SIZE), dtype=np.float32)
    rel[fg_grid] = np.clip((dist_grid[fg_grid] - center_z) / base.REL_DISTANCE_SCALE_MM, -3.0, 3.0)
    feature[0] = rel

    abs_norm = np.zeros((base.GRID_SIZE, base.GRID_SIZE), dtype=np.float32)
    abs_norm[fg_grid] = np.clip(
        (dist_grid[fg_grid] - base.DISTANCE_MIN_MM) / (base.DISTANCE_MAX_MM - base.DISTANCE_MIN_MM),
        0.0,
        1.0,
    )
    feature[1] = abs_norm
    feature[2] = valid_grid.astype(np.float32)
    feature[3] = fg_grid.astype(np.float32)
    feature[4] = status_ok.reshape(base.GRID_SIZE, base.GRID_SIZE).astype(np.float32)
    feature[5] = nb_norm.reshape(base.GRID_SIZE, base.GRID_SIZE)

    cnh_matrix = cnh.reshape(ZONE_COUNT, CNH_BINS).astype(np.float32)
    cnh_feature = np.zeros_like(cnh_matrix)
    cnh_feature[foreground_mask, :] = cnh_matrix[foreground_mask, :]
    cnh_scale = float(np.max(np.abs(cnh_feature[foreground_mask, :])))
    if cnh_scale < base.CNH_EPS:
        cnh_scale = 1.0
    cnh_feature /= cnh_scale
    for bin_idx in range(CNH_BINS):
        feature[6 + bin_idx] = cnh_feature[:, bin_idx].reshape(base.GRID_SIZE, base.GRID_SIZE)

    # 全局量复制到每个 zone，保持固定输入尺寸，便于 CubeAI 部署。
    feature[14, :, :] = np.clip((center_x - 3.5) / 3.5, -1.0, 1.0)
    feature[15, :, :] = np.clip((center_y - 3.5) / 3.5, -1.0, 1.0)
    feature[16, :, :] = np.clip((center_z - base.Z_CENTER_MM) / base.Z_SCALE_MM, -2.0, 2.0)
    feature[17, :, :] = foreground_count / float(ZONE_COUNT)
    feature[18, :, :] = bbox_width / float(base.GRID_SIZE)
    feature[19, :, :] = bbox_height / float(base.GRID_SIZE)

    frame_meta = {
        "center_x": center_x,
        "center_y": center_y,
        "center_z": center_z,
        "hand_zone_count": foreground_count,
        "bbox_width": bbox_width,
        "bbox_height": bbox_height,
        "z_bin": int(np.digitize(center_z, [220.0, 280.0, 340.0, 420.0])),
    }
    frame_meta.update(roi_meta)
    return feature, frame_meta, None


_original_read_dataset_cnn = base.read_dataset_cnn


def read_dataset_cnn_roi(*args, **kwargs):
    """复用原数据读取逻辑，只修正报告中的前处理描述。"""
    x, y, meta = _original_read_dataset_cnn(*args, **kwargs)
    meta["preprocess"].update(
        {
            "mode": "cnn_20ch_roi_foreground_relative",
            "roi_tip_is_top": ROI_TIP_IS_TOP,
            "roi_palm_width_mm": ROI_PALM_WIDTH_MM,
            "roi_palm_height_mm": ROI_PALM_HEIGHT_MM,
            "roi_tan_half_fov": ROI_TAN_HALF_FOV,
            "roi_rows_for_z_ref": ROI_ROWS_FOR_Z_REF,
            "roi_source": "raw_foreground(d_min+margin) clipped by tip anchored palm rectangle",
        }
    )
    return x, y, meta


# 只替换前处理和报告信息，网络结构、训练参数、导出流程全部沿用原脚本。
base.preprocess_frame_cnn = preprocess_frame_cnn_roi
base.read_dataset_cnn = read_dataset_cnn_roi


if __name__ == "__main__":
    base.main()
