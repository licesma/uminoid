"""Typed Psi-0 finetune config that renders to CLI args for upstream scripts/train.py.

Mirrors the flag set in third_party/Humanoid-Exo-Learning/scripts/train/psi0/0528/finetune-pick-plum-may-28.sh
so that the rendered torchrun command is functionally identical to Sicheng's bash version,
but tasks can subclass and override single fields instead of copy-pasting 50 lines.
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from pathlib import Path


def _psi_home() -> str:
    home = os.environ.get("PSI_HOME")
    if not home:
        raise RuntimeError(
            "PSI_HOME is not set. Export it or `set -a; source .env; set +a` before running."
        )
    return home


@dataclass
class TrainSection:
    name: str = "finetune"
    data_parallel: str = "ddp"
    mixed_precision: str = "bf16"
    train_batch_size: int = 16
    max_checkpoints_to_keep: int = 10
    gradient_accumulation_steps: int = 4
    learning_rate: float = 1e-4
    max_training_steps: int = 20000
    warmup_ratio: str = "None"  # draccus parses literal "None" -> Python None
    warmup_steps: int = 500
    checkpointing_steps: int = 2500
    validation_steps: int = 1000
    val_num_batches: int = 20
    max_grad_norm: float = 1.0
    lr_scheduler_type: str = "cosine"
    weight_decay: float = 1e-6
    betas: tuple[float, float] = (0.95, 0.999)
    resume_from_checkpoint: str | None = None


@dataclass
class DataSection:
    # Resolved relative to the cwd of the torchrun subprocess, which run.py
    # sets to ROOT_DIR. So this points at <repo>/training_data/.
    root_dir: str = "training_data"
    train_repo_ids: str = ""  # required, set per-task; matches a subdir under root_dir

    repack_pad_action_dim: int = 36
    repack_pad_state_dim: int = 36

    stat_path: str = "meta/stats_psi0.json"
    stat_action_key: str = "action"
    stat_state_key: str = "states"
    action_norm_type: str = "bounds"
    use_norm_mask: bool = False
    normalize_state: bool = True
    pad_action_dim: int = 36
    pad_state_dim: int = 36

    img_aug: bool = True
    resize_size: tuple[int, int] = (240, 320)
    center_crop_size: tuple[int, int] = (240, 320)


@dataclass
class ModelSection:
    model_name_or_path: str = ""  # filled in __post_init__ from PSI_HOME
    pretrained_action_header_path: str = ""

    noise_scheduler: str = "flow"
    train_diffusion_steps: int = 1000
    n_conditions: int = 0
    action_chunk_size: int = 30
    action_dim: int = 36
    action_exec_horizon: int = 30
    observation_horizon: int = 1
    odim: int = 36
    view_feature_dim: int = 2048

    tune_vlm: bool = False
    use_film: bool = False
    combined_temb: bool = False
    rtc: bool = True
    max_delay: int = 8


@dataclass
class LogSection:
    report_to: str = "wandb"


@dataclass
class FinetuneConfig:
    """One finetune run. Subclass (or build via a factory function) to override fields."""

    config_name: str = "finetune_real_psi0_config"  # upstream registered config
    seed: int = 292285
    experiment: str = ""  # required; rendered as `--exp=...` for upstream
    # Reuse a prior run's timestamp so a resumed run writes to the same folder
    # (the run dir name ends in this). Leave None for a fresh run.
    timestamp: str | None = None

    train: TrainSection = field(default_factory=TrainSection)
    data: DataSection = field(default_factory=DataSection)
    model: ModelSection = field(default_factory=ModelSection)
    log: LogSection = field(default_factory=LogSection)

    def __post_init__(self) -> None:
        psi_home = _psi_home()
        if not self.model.model_name_or_path:
            self.model.model_name_or_path = (
                f"{psi_home}/cache/checkpoints/psi0/"
                "pre.fast.1by1.2601091803.ckpt.ego200k.he30k"
            )
        if not self.model.pretrained_action_header_path:
            self.model.pretrained_action_header_path = (
                f"{psi_home}/cache/checkpoints/psi0/"
                "postpre.1by1.pad36.2601131206.ckpt.he30k"
            )

    def to_cli_args(self) -> list[str]:
        if not self.experiment:
            raise ValueError("FinetuneConfig.experiment must be set")
        if not self.data.train_repo_ids:
            raise ValueError("FinetuneConfig.data.train_repo_ids must be set")

        a: list[str] = [
            self.config_name,
            f"--seed={self.seed}",
            f"--exp={self.experiment}",
        ]
        if self.timestamp is not None:
            a.append(f"--timestamp={self.timestamp}")

        t = self.train
        a += [
            f"--train.name={t.name}",
            f"--train.data_parallel={t.data_parallel}",
            f"--train.mixed_precision={t.mixed_precision}",
            f"--train.train_batch_size={t.train_batch_size}",
            f"--train.max_checkpoints_to_keep={t.max_checkpoints_to_keep}",
            f"--train.gradient_accumulation_steps={t.gradient_accumulation_steps}",
            f"--train.learning_rate={t.learning_rate}",
            f"--train.max_training_steps={t.max_training_steps}",
            f"--train.warmup_ratio={t.warmup_ratio}",
            f"--train.warmup_steps={t.warmup_steps}",
            f"--train.checkpointing_steps={t.checkpointing_steps}",
            f"--train.validation_steps={t.validation_steps}",
            f"--train.val_num_batches={t.val_num_batches}",
            f"--train.max_grad_norm={t.max_grad_norm}",
            f"--train.lr_scheduler_type={t.lr_scheduler_type}",
            f"--train.lr_scheduler_kwargs.weight_decay={t.weight_decay}",
            "--train.lr_scheduler_kwargs.betas",
            str(t.betas[0]),
            str(t.betas[1]),
        ]
        if t.resume_from_checkpoint is not None:
            a.append(f"--train.resume_from_checkpoint={t.resume_from_checkpoint}")

        a += [f"--log.report_to={self.log.report_to}"]

        d = self.data
        a += [
            f"--data.root_dir={d.root_dir}",
            f"--data.train_repo_ids={d.train_repo_ids}",
            f"--data.transform.repack.pad-action-dim={d.repack_pad_action_dim}",
            f"--data.transform.repack.pad-state-dim={d.repack_pad_state_dim}",
            f"--data.transform.field.stat-path={d.stat_path}",
            f"--data.transform.field.stat-action-key={d.stat_action_key}",
            f"--data.transform.field.stat-state-key={d.stat_state_key}",
            f"--data.transform.field.action_norm_type={d.action_norm_type}",
            "--data.transform.field.use-norm-mask"
            if d.use_norm_mask
            else "--data.transform.field.no-use-norm-mask",
        ]
        if d.normalize_state:
            a.append("--data.transform.field.normalize-state")
        a += [
            f"--data.transform.field.pad-action-dim={d.pad_action_dim}",
            f"--data.transform.field.pad-state-dim={d.pad_state_dim}",
        ]
        if d.img_aug:
            a.append("--data.transform.model.img-aug")
        a += [
            "--data.transform.model.resize.size",
            str(d.resize_size[0]),
            str(d.resize_size[1]),
            "--data.transform.model.center_crop.size",
            str(d.center_crop_size[0]),
            str(d.center_crop_size[1]),
        ]

        m = self.model
        a += [
            f"--model.model_name_or_path={m.model_name_or_path}",
            f"--model.pretrained_action_header_path={m.pretrained_action_header_path}",
            f"--model.noise-scheduler={m.noise_scheduler}",
            f"--model.train-diffusion-steps={m.train_diffusion_steps}",
            f"--model.n_conditions={m.n_conditions}",
            f"--model.action-chunk-size={m.action_chunk_size}",
            f"--model.action-dim={m.action_dim}",
            f"--model.action-exec-horizon={m.action_exec_horizon}",
            f"--model.observation-horizon={m.observation_horizon}",
            f"--model.odim={m.odim}",
            f"--model.view_feature_dim={m.view_feature_dim}",
            "--model.tune-vlm" if m.tune_vlm else "--model.no-tune-vlm",
            "--model.use_film" if m.use_film else "--model.no-use_film",
            "--model.combined_temb" if m.combined_temb else "--model.no-combined_temb",
        ]
        if m.rtc:
            a.append("--model.rtc")
        a.append(f"--model.max-delay={m.max_delay}")

        return a
