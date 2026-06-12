from __future__ import annotations

from py.train.config import FinetuneConfig


def full_june_8() -> FinetuneConfig:
    cfg = FinetuneConfig(experiment="apple_full_june_8")
    cfg.data.train_repo_ids = "apple_june_8"
    cfg.train.max_training_steps = 40000
    return cfg


def f80_june_8() -> FinetuneConfig:
    cfg = FinetuneConfig(experiment="apple_first_80_june_8")
    cfg.data.train_repo_ids = "apple_june_8"
    cfg.data.train_fraction = 0.8
    cfg.train.max_training_steps = 40000
    return cfg
