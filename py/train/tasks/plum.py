from __future__ import annotations

from py.train.config import FinetuneConfig


def plum_may_28() -> FinetuneConfig:
    cfg = FinetuneConfig(experiment="pick_palm_place_box_may_28_plum")
    cfg.data.train_repo_ids = "may_28_plum_v"
    cfg.train.max_training_steps = 40000
    cfg.timestamp = "2606061116"
    
    cfg.train.resume_from_checkpoint = (
        ".runs/finetune/pick_palm_place_box_may_28_plum"
        ".real.flow1000.cosine.lr1.0e-04.b64.gpus1.2606061116"
    )
    return cfg
