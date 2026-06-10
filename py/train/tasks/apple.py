from __future__ import annotations

from py.train.config import FinetuneConfig


def full_june_8() -> FinetuneConfig:
    cfg = FinetuneConfig(experiment="apple_full_june_8")
    cfg.data.train_repo_ids = "may_28_plum_v"
    cfg.train.max_training_steps = 40000
    return cfg
