"""直接监督或蒸馏训练Cube.AI兼容的Tiny ToF-TCN。"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time
from typing import Any

import numpy as np
import torch
from torch.utils.data import DataLoader
from tqdm.auto import tqdm

from training.dataset import EpisodeWindowDataset
from training.config import MOTOR_COUNT
from training.losses import compute_act_loss
from training.model import TinyTofAct, count_parameters
from training.normalization import NormalizationStats, fit_normalization
from training.tcn_model import TinyTofTcn
from training.train import (
    build_scene_sqrt_sampler,
    build_warmup_cosine_scheduler,
    discover_episodes,
    format_duration,
    load_episode_split,
    move_batch,
    seed_data_worker,
    set_random_seed,
    split_episodes,
)


# 与H7当前部署策略一致，评价每个chunk中的未来第3帧目标。
TEMPORAL_HORIZON_INDEX = 2
# 低于10q的变化主要来自量化和传感器波动，不计为有效换向。
TEMPORAL_FLIP_THRESHOLD_Q = 10.0


def calculate_temporal_flip_metrics(
    prediction_q: np.ndarray,
    target_q: np.ndarray,
    valid: np.ndarray,
    windows: list[Any],
) -> dict[str, float | list[float]]:
    """统计连续验证窗口的目标跳变和方向翻转。

    Args:
        prediction_q: ``[N,3]``预测目标，单位0.01rad。
        target_q: ``[N,3]``真实目标，单位0.01rad。
        valid: ``[N]``，True表示该窗口第3帧目标有效。
        windows: 与前三个数组同序的WindowIndex列表。

    Returns:
        预测/真实翻转率、预测跳变量及模型额外产生的翻转率。

    Note:
        只有同一episode且current_frame连续的窗口才参与统计。翻转率的
        分母是连续两次变化量都超过阈值的有效换向机会，不包含静止抖动。
    """

    expected_shape = (len(windows), MOTOR_COUNT)
    if prediction_q.shape != expected_shape or target_q.shape != expected_shape:
        raise ValueError(f"时序目标尺寸必须为{expected_shape}")
    if valid.shape != (len(windows),):
        raise ValueError("时序目标有效掩码尺寸错误")

    prediction_flips = np.zeros(MOTOR_COUNT, dtype=np.int64)
    prediction_opportunities = np.zeros(MOTOR_COUNT, dtype=np.int64)
    target_flips = np.zeros(MOTOR_COUNT, dtype=np.int64)
    target_opportunities = np.zeros(MOTOR_COUNT, dtype=np.int64)
    prediction_jumps: list[np.ndarray] = []

    previous_prediction: np.ndarray | None = None
    previous_target: np.ndarray | None = None
    previous_prediction_delta: np.ndarray | None = None
    previous_target_delta: np.ndarray | None = None
    previous_episode = -1
    previous_frame = -1

    for index, window in enumerate(windows):
        consecutive = (
            bool(valid[index])
            and window.episode_index == previous_episode
            and window.current_frame == previous_frame + 1
        )
        if not consecutive:
            previous_prediction_delta = None
            previous_target_delta = None
        else:
            assert previous_prediction is not None
            assert previous_target is not None
            prediction_delta = prediction_q[index] - previous_prediction
            target_delta = target_q[index] - previous_target
            prediction_jumps.append(np.abs(prediction_delta))

            if previous_prediction_delta is not None:
                active = (
                    (np.abs(prediction_delta) > TEMPORAL_FLIP_THRESHOLD_Q)
                    & (np.abs(previous_prediction_delta) > TEMPORAL_FLIP_THRESHOLD_Q)
                )
                prediction_opportunities += active
                prediction_flips += active & (
                    np.sign(prediction_delta) != np.sign(previous_prediction_delta)
                )
            if previous_target_delta is not None:
                active = (
                    (np.abs(target_delta) > TEMPORAL_FLIP_THRESHOLD_Q)
                    & (np.abs(previous_target_delta) > TEMPORAL_FLIP_THRESHOLD_Q)
                )
                target_opportunities += active
                target_flips += active & (
                    np.sign(target_delta) != np.sign(previous_target_delta)
                )

            previous_prediction_delta = prediction_delta
            previous_target_delta = target_delta

        if bool(valid[index]):
            previous_prediction = prediction_q[index]
            previous_target = target_q[index]
            previous_episode = window.episode_index
            previous_frame = window.current_frame
        else:
            previous_prediction = None
            previous_target = None
            previous_episode = -1
            previous_frame = -1

    prediction_rate_by_motor = np.divide(
        prediction_flips,
        np.maximum(prediction_opportunities, 1),
    )
    target_rate_by_motor = np.divide(
        target_flips,
        np.maximum(target_opportunities, 1),
    )
    prediction_rate = float(
        prediction_flips.sum() / max(int(prediction_opportunities.sum()), 1)
    )
    target_rate = float(
        target_flips.sum() / max(int(target_opportunities.sum()), 1)
    )
    jump = (
        np.concatenate(prediction_jumps).astype(np.float64, copy=False)
        if prediction_jumps else np.zeros(1, dtype=np.float64)
    )
    return {
        "prediction_flip_rate": prediction_rate,
        "target_flip_rate": target_rate,
        "excess_flip_rate": max(prediction_rate - target_rate, 0.0),
        "prediction_flip_rate_by_motor": prediction_rate_by_motor.tolist(),
        "target_flip_rate_by_motor": target_rate_by_motor.tolist(),
        "prediction_jump_mean_q": float(jump.mean()),
        "prediction_jump_p95_q": float(np.percentile(jump, 95)),
    }


def masked_mae_q(
    prediction: torch.Tensor,
    target: torch.Tensor,
    padding_mask: torch.Tensor,
    action_std: torch.Tensor,
) -> torch.Tensor:
    """计算有效动作位置的平均绝对误差，单位为0.01rad。

    Args:
        prediction: 归一化的``[B,10,3]``模型输出。
        target: 归一化的``[B,10,3]``真实未来位置。
        padding_mask: ``[B,10]``，True表示该帧不参与评价。
        action_std: ``[1,1,3]``动作归一化标准差。

    Returns:
        单个标量，表示三个电机所有有效未来帧的MAE(q)。
    """

    valid = (~padding_mask).unsqueeze(-1).expand_as(prediction)
    error_q = torch.abs((prediction - target) * action_std)
    return error_q[valid].mean()


def train_one_epoch(
    student: TinyTofTcn,
    teacher: TinyTofAct | None,
    loader: DataLoader,
    optimizer: torch.optim.Optimizer,
    scheduler: torch.optim.lr_scheduler.LRScheduler,
    device: torch.device,
    teacher_weight: float,
    smooth_weight: float,
    action_std: torch.Tensor,
    epoch: int,
    epochs: int,
    global_step: int,
    wandb_run: Any | None,
) -> tuple[dict[str, float], int]:
    """训练一个epoch；teacher为None时只使用真实未来位置监督。"""

    student.train()
    totals = {"total": 0.0, "ground_truth": 0.0, "teacher": 0.0, "mae_q": 0.0}
    sample_count = 0
    progress = tqdm(loader, desc=f"训练 {epoch}/{epochs}", unit="batch", leave=False)
    for raw_batch in progress:
        batch = move_batch(raw_batch, device)
        optimizer.zero_grad(set_to_none=True)
        prediction = student(batch["tof_history"], batch["state_history"])
        ground_truth_loss = compute_act_loss(
            prediction,
            batch["action_chunk"],
            batch["padding_mask"],
            smooth_weight=smooth_weight,
            kl_weight=0.0,
        )
        if teacher is None:
            teacher_loss = prediction.new_zeros(())
            total_loss = ground_truth_loss.total
        else:
            with torch.no_grad():
                teacher_prediction = teacher(
                    batch["tof_history"],
                    batch["state_history"],
                )
            teacher_loss = compute_act_loss(
                prediction,
                teacher_prediction,
                batch["padding_mask"],
                smooth_weight=0.0,
                kl_weight=0.0,
            ).action
            total_loss = (
                (1.0 - teacher_weight) * ground_truth_loss.total
                + teacher_weight * teacher_loss
            )
        total_loss.backward()
        gradient_norm = torch.nn.utils.clip_grad_norm_(student.parameters(), 1.0)
        learning_rate = optimizer.param_groups[0]["lr"]
        optimizer.step()
        scheduler.step()
        global_step += 1

        mae_q = masked_mae_q(
            prediction.detach(),
            batch["action_chunk"],
            batch["padding_mask"],
            action_std,
        )
        batch_size = prediction.shape[0]
        sample_count += batch_size
        totals["total"] += float(total_loss.detach()) * batch_size
        totals["ground_truth"] += float(ground_truth_loss.total.detach()) * batch_size
        totals["teacher"] += float(teacher_loss.detach()) * batch_size
        totals["mae_q"] += float(mae_q) * batch_size
        progress.set_postfix(loss=f"{totals['total'] / sample_count:.5f}", refresh=False)

        if wandb_run is not None:
            wandb_run.log(
                {
                    "train/global_step": global_step,
                    "loss/train_step_total": float(total_loss.detach()),
                    "loss/train_step_ground_truth": float(ground_truth_loss.total.detach()),
                    "loss/train_step_teacher": float(teacher_loss.detach()),
                    "metric/train_step_mae_q": float(mae_q),
                    "train/gradient_norm": float(gradient_norm),
                    "optimizer/step_learning_rate": learning_rate,
                }
            )
    return {name: value / sample_count for name, value in totals.items()}, global_step


@torch.no_grad()
def validate(
    student: TinyTofTcn,
    loader: DataLoader,
    device: torch.device,
    smooth_weight: float,
    action_mean: torch.Tensor,
    action_std: torch.Tensor,
    epoch: int,
    epochs: int,
) -> dict[str, float | list[float]]:
    """根据真实未来位置评价精度，并统计连续目标方向翻转。"""

    student.eval()
    totals = {"total": 0.0, "action": 0.0, "smooth": 0.0, "mae_q": 0.0}
    sample_count = 0
    prediction_horizon: list[np.ndarray] = []
    target_horizon: list[np.ndarray] = []
    horizon_valid: list[np.ndarray] = []
    progress = tqdm(loader, desc=f"验证 {epoch}/{epochs}", unit="batch", leave=False)
    for raw_batch in progress:
        batch = move_batch(raw_batch, device)
        prediction = student(batch["tof_history"], batch["state_history"])
        loss = compute_act_loss(
            prediction,
            batch["action_chunk"],
            batch["padding_mask"],
            smooth_weight=smooth_weight,
            kl_weight=0.0,
        )
        mae_q = masked_mae_q(
            prediction,
            batch["action_chunk"],
            batch["padding_mask"],
            action_std,
        )
        batch_size = prediction.shape[0]
        sample_count += batch_size
        totals["total"] += float(loss.total) * batch_size
        totals["action"] += float(loss.action) * batch_size
        totals["smooth"] += float(loss.smooth) * batch_size
        totals["mae_q"] += float(mae_q) * batch_size
        prediction_q = prediction * action_std + action_mean
        target_q = batch["action_chunk"] * action_std + action_mean
        prediction_horizon.append(
            prediction_q[:, TEMPORAL_HORIZON_INDEX].detach().cpu().numpy()
        )
        target_horizon.append(
            target_q[:, TEMPORAL_HORIZON_INDEX].detach().cpu().numpy()
        )
        horizon_valid.append(
            (~batch["padding_mask"][:, TEMPORAL_HORIZON_INDEX]).cpu().numpy()
        )

    metrics: dict[str, float | list[float]] = {
        name: value / sample_count for name, value in totals.items()
    }
    if not isinstance(loader.dataset, EpisodeWindowDataset):
        raise TypeError("时序翻转评价要求EpisodeWindowDataset")
    metrics.update(
        calculate_temporal_flip_metrics(
            np.concatenate(prediction_horizon),
            np.concatenate(target_horizon),
            np.concatenate(horizon_valid),
            loader.dataset.windows,
        )
    )
    return metrics


def build_argument_parser() -> argparse.ArgumentParser:
    """定义TCN训练参数；默认使用真实标签直接监督。"""

    parser = argparse.ArgumentParser(description="训练Tiny ToF-TCN")
    parser.add_argument(
        "--training-mode",
        choices=("direct", "distill"),
        default="direct",
        help="direct仅使用真实标签；distill额外使用Transformer教师输出",
    )
    parser.add_argument("--data-dir", type=Path, required=True)
    parser.add_argument(
        "--split-file",
        type=Path,
        default=None,
        help="清洗阶段生成的固定split.json；未指定时保持原随机划分",
    )
    parser.add_argument(
        "--teacher-run-dir",
        type=Path,
        default=None,
        help="distill模式必填，目录内应包含stats.json和教师checkpoint",
    )
    parser.add_argument("--teacher-checkpoint", type=Path, default=None)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--learning-rate", type=float, default=5.0e-4)
    parser.add_argument("--minimum-lr", type=float, default=1.0e-5)
    parser.add_argument("--warmup-ratio", type=float, default=0.05)
    parser.add_argument("--weight-decay", type=float, default=1.0e-4)
    parser.add_argument("--teacher-weight", type=float, default=0.40)
    parser.add_argument("--smooth-weight", type=float, default=0.10)
    parser.add_argument("--checkpoint-interval", type=int, default=10)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--num-workers", type=int, default=0)
    parser.add_argument("--preload-data", action="store_true")
    parser.add_argument("--augment-data", action="store_true")
    parser.add_argument(
        "--train-window-stride",
        type=int,
        default=1,
        help="训练窗口起点步长；验证集始终保持步长1",
    )
    parser.add_argument(
        "--scene-sqrt-sampling",
        action="store_true",
        help="按场景窗口数的平方根比例采样，仅作用于训练集",
    )
    parser.add_argument(
        "--flip-score-weight",
        type=float,
        default=1.0,
        help="额外翻转率对best选模分数的惩罚权重",
    )
    parser.add_argument("--device", choices=("auto", "cpu", "cuda"), default="auto")
    parser.add_argument("--wandb-mode", choices=("disabled", "online", "offline"), default="disabled")
    parser.add_argument("--wandb-project", default="flexible-tentacle-act")
    parser.add_argument("--wandb-run-name", default="tcn-distill-v1")
    return parser


def main() -> None:
    """完成数据划分、可选教师加载、TCN训练和checkpoint保存。"""

    args = build_argument_parser().parse_args()
    if args.epochs <= 0 or args.batch_size <= 0:
        raise ValueError("epochs和batch-size必须大于0")
    if not 0.0 <= args.teacher_weight < 1.0:
        raise ValueError("teacher-weight必须位于[0,1)")
    if args.training_mode == "distill" and args.teacher_run_dir is None:
        raise ValueError("distill模式必须提供--teacher-run-dir")
    if args.training_mode == "direct" and (
        args.teacher_run_dir is not None or args.teacher_checkpoint is not None
    ):
        raise ValueError("direct模式不能提供教师模型参数")
    if args.checkpoint_interval < 0:
        raise ValueError("checkpoint-interval不能为负数")
    if args.train_window_stride <= 0:
        raise ValueError("train-window-stride必须大于0")
    if args.flip_score_weight < 0.0:
        raise ValueError("flip-score-weight不能为负数")
    set_random_seed(args.seed)

    if args.device == "cuda" and not torch.cuda.is_available():
        raise RuntimeError("请求使用CUDA，但当前PyTorch未检测到CUDA")
    device = torch.device(
        "cuda" if args.device == "auto" and torch.cuda.is_available() else
        ("cpu" if args.device == "auto" else args.device)
    )

    if args.split_file is None:
        episode_paths = discover_episodes(args.data_dir)
        train_paths, val_paths, test_paths = split_episodes(episode_paths, args.seed)
    else:
        train_paths, val_paths, test_paths = load_episode_split(args.split_file)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / "split.json").write_text(
        json.dumps(
            {
                "seed": args.seed,
                "train": [str(path.resolve()) for path in train_paths],
                "validation": [str(path.resolve()) for path in val_paths],
                "test": [str(path.resolve()) for path in test_paths],
            },
            ensure_ascii=False,
            indent=2,
        ),
        encoding="utf-8",
    )

    if args.training_mode == "direct":
        # 只使用训练集计算统计量，避免验证集和测试集信息泄漏。
        stats = fit_normalization(train_paths)
    else:
        assert args.teacher_run_dir is not None
        # 蒸馏时教师和学生必须使用相同统计量，确保输出坐标系一致。
        stats = NormalizationStats.load(args.teacher_run_dir / "stats.json")
    stats.save(args.output_dir / "stats.json")
    train_dataset = EpisodeWindowDataset(
        train_paths,
        stats,
        preload=args.preload_data,
        augment=args.augment_data,
        window_stride=args.train_window_stride,
    )
    val_dataset = EpisodeWindowDataset(val_paths, stats, preload=args.preload_data)
    generator = torch.Generator().manual_seed(args.seed)
    train_sampler = None
    scene_window_counts: dict[int, int] = {}
    if args.scene_sqrt_sampling:
        train_sampler, scene_window_counts = build_scene_sqrt_sampler(
            train_dataset,
            generator,
        )
    train_loader = DataLoader(
        train_dataset,
        batch_size=args.batch_size,
        shuffle=train_sampler is None,
        sampler=train_sampler,
        num_workers=args.num_workers,
        pin_memory=device.type == "cuda",
        worker_init_fn=seed_data_worker,
        generator=generator,
    )
    val_loader = DataLoader(
        val_dataset,
        batch_size=args.batch_size,
        shuffle=False,
        num_workers=args.num_workers,
        pin_memory=device.type == "cuda",
    )

    teacher: TinyTofAct | None = None
    teacher_path: Path | None = None
    if args.training_mode == "distill":
        assert args.teacher_run_dir is not None
        teacher_path = args.teacher_checkpoint or (args.teacher_run_dir / "best.pt")
        teacher_checkpoint = torch.load(
            teacher_path,
            map_location=device,
            weights_only=False,
        )
        teacher = TinyTofAct().to(device)
        teacher.load_state_dict(teacher_checkpoint["model_state"])
        teacher.eval()
        teacher.requires_grad_(False)

    student = TinyTofTcn().to(device)
    if teacher is not None:
        student.initialize_from_teacher(teacher)
    optimizer = torch.optim.AdamW(
        student.parameters(),
        lr=args.learning_rate,
        weight_decay=args.weight_decay,
    )
    total_steps = args.epochs * len(train_loader)
    scheduler = build_warmup_cosine_scheduler(
        optimizer,
        total_steps,
        args.learning_rate,
        args.warmup_ratio,
        args.learning_rate * 0.20,
        args.minimum_lr,
    )
    action_std = torch.as_tensor(
        stats.action_std,
        dtype=torch.float32,
        device=device,
    ).reshape(1, 1, MOTOR_COUNT)
    action_mean = torch.as_tensor(
        stats.action_mean,
        dtype=torch.float32,
        device=device,
    ).reshape(1, 1, MOTOR_COUNT)

    wandb_run = None
    if args.wandb_mode != "disabled":
        import wandb

        wandb_config = {
            name: (str(value) if isinstance(value, Path) else value)
            for name, value in vars(args).items()
        }
        wandb_config["parameter_count"] = count_parameters(student)
        wandb_config["scene_window_counts"] = scene_window_counts
        wandb_run = wandb.init(
            project=args.wandb_project,
            name=args.wandb_run_name,
            mode=args.wandb_mode,
            dir=str(args.output_dir),
            config=wandb_config,
        )
        wandb_run.define_metric("train/global_step")
        wandb_run.define_metric("loss/train_step_*", step_metric="train/global_step")
        wandb_run.define_metric("metric/train_step_*", step_metric="train/global_step")
        wandb_run.define_metric("epoch")
        wandb_run.define_metric("loss/train_epoch_*", step_metric="epoch")
        wandb_run.define_metric("loss/validation_*", step_metric="epoch")
        wandb_run.define_metric("metric/validation_*", step_metric="epoch")

    print(
        f"mode={args.training_mode} device={device} "
        f"student_parameters={count_parameters(student)} "
        f"episodes={len(train_paths)}/{len(val_paths)}/{len(test_paths)} "
        f"windows={len(train_dataset)}/{len(val_dataset)} "
        f"train_stride={args.train_window_stride} "
        f"scene_sqrt_sampling={args.scene_sqrt_sampling}"
    )
    if scene_window_counts:
        print(f"train_scene_windows={dict(sorted(scene_window_counts.items()))}")
    history: list[dict[str, object]] = []
    best_validation_mae_q = float("inf")
    best_selection_score = float("inf")
    global_step = 0
    start_time = time.perf_counter()
    epoch_durations: list[float] = []
    for epoch in tqdm(range(1, args.epochs + 1), desc="总进度", unit="epoch"):
        epoch_start = time.perf_counter()
        train_metrics, global_step = train_one_epoch(
            student,
            teacher,
            train_loader,
            optimizer,
            scheduler,
            device,
            args.teacher_weight,
            args.smooth_weight,
            action_std,
            epoch,
            args.epochs,
            global_step,
            wandb_run,
        )
        val_metrics = validate(
            student,
            val_loader,
            device,
            args.smooth_weight,
            action_mean,
            action_std,
            epoch,
            args.epochs,
        )
        epoch_seconds = time.perf_counter() - epoch_start
        epoch_durations.append(epoch_seconds)
        elapsed = time.perf_counter() - start_time
        remaining = float(np.mean(epoch_durations[-5:])) * (args.epochs - epoch)
        selection_score = float(val_metrics["mae_q"]) * (
            1.0 + args.flip_score_weight * float(val_metrics["excess_flip_rate"])
        )
        val_metrics["selection_score"] = selection_score
        # best同时考虑位置精度和模型比真实轨迹额外产生的方向翻转。
        is_best = selection_score < best_selection_score
        best_selection_score = min(best_selection_score, selection_score)
        best_validation_mae_q = min(
            best_validation_mae_q,
            float(val_metrics["mae_q"]),
        )
        record = {"epoch": epoch, "train": train_metrics, "validation": val_metrics}
        history.append(record)

        checkpoint = {
            "epoch": epoch,
            "model_state": student.state_dict(),
            "optimizer_state": optimizer.state_dict(),
            "scheduler_state": scheduler.state_dict(),
            "global_step": global_step,
            "validation_loss": val_metrics["total"],
            "validation_mae_q": val_metrics["mae_q"],
            "validation_prediction_flip_rate": val_metrics["prediction_flip_rate"],
            "validation_target_flip_rate": val_metrics["target_flip_rate"],
            "validation_excess_flip_rate": val_metrics["excess_flip_rate"],
            "validation_selection_score": selection_score,
            "flip_score_weight": args.flip_score_weight,
            "train_window_stride": args.train_window_stride,
            "scene_sqrt_sampling": args.scene_sqrt_sampling,
            "scene_window_counts": scene_window_counts,
            "training_mode": args.training_mode,
            "teacher_checkpoint": (
                str(teacher_path.resolve()) if teacher_path is not None else None
            ),
            "teacher_weight": (
                args.teacher_weight if teacher is not None else 0.0
            ),
            "parameter_count": count_parameters(student),
        }
        torch.save(checkpoint, args.output_dir / "last.pt")
        if is_best:
            torch.save(checkpoint, args.output_dir / "best.pt")
        if args.checkpoint_interval > 0 and epoch % args.checkpoint_interval == 0:
            torch.save(checkpoint, args.output_dir / f"epoch_{epoch:03d}.pt")
        (args.output_dir / "history.json").write_text(
            json.dumps(history, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )

        if wandb_run is not None:
            wandb_run.log(
                {
                    "epoch": epoch,
                    "loss/train_epoch_total": train_metrics["total"],
                    "loss/train_epoch_ground_truth": train_metrics["ground_truth"],
                    "loss/train_epoch_teacher": train_metrics["teacher"],
                    "metric/train_epoch_mae_q": train_metrics["mae_q"],
                    "loss/validation_total": val_metrics["total"],
                    "loss/validation_action": val_metrics["action"],
                    "loss/validation_smooth": val_metrics["smooth"],
                    "metric/validation_mae_q": val_metrics["mae_q"],
                    "metric/validation_prediction_flip_rate": val_metrics[
                        "prediction_flip_rate"
                    ],
                    "metric/validation_target_flip_rate": val_metrics[
                        "target_flip_rate"
                    ],
                    "metric/validation_excess_flip_rate": val_metrics[
                        "excess_flip_rate"
                    ],
                    "metric/validation_prediction_jump_p95_q": val_metrics[
                        "prediction_jump_p95_q"
                    ],
                    "metric/validation_selection_score": selection_score,
                    "metric/best_validation_mae_q": best_validation_mae_q,
                    "metric/best_selection_score": best_selection_score,
                    "time/epoch_seconds": epoch_seconds,
                    "time/elapsed_seconds": elapsed,
                    "time/remaining_seconds": remaining,
                }
            )
        tqdm.write(
            f"epoch={epoch:03d} train={train_metrics['total']:.6f} "
            f"val={val_metrics['total']:.6f} mae={val_metrics['mae_q']:.1f}q "
            f"flip={100.0 * float(val_metrics['prediction_flip_rate']):.1f}% "
            f"excess={100.0 * float(val_metrics['excess_flip_rate']):.1f}% "
            f"score={selection_score:.1f} best={best_selection_score:.1f} "
            f"time={format_duration(epoch_seconds)} "
            f"ETA={format_duration(remaining)}"
        )

    if wandb_run is not None:
        wandb_run.summary["best_validation_mae_q"] = best_validation_mae_q
        wandb_run.summary["best_selection_score"] = best_selection_score
        wandb_run.finish()


if __name__ == "__main__":
    main()
