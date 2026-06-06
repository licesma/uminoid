from __future__ import annotations

from py.train.config import FinetuneConfig


def pen_marker_may_26() -> FinetuneConfig:
    cfg = FinetuneConfig(experiment="pick_marker_pen_place_box_0525")
    # cfg.data.train_repo_ids = "may_25_???"  # TBD: which may_25 subdir
    return cfg


def pen_marker_with_actions_may_28() -> FinetuneConfig:
    cfg = FinetuneConfig(experiment="pick_marker_pen_place_box_may_25_with_actions")
    cfg.data.train_repo_ids = "may_25_with_actions"
    return cfg
