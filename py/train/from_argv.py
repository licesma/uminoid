"""Reverse of FinetuneConfig.to_cli_args.

Parse an argv.txt produced by a past Psi-0 finetune run and emit a Python
factory function with only the fields that differ from FinetuneConfig's
defaults.

Usage:
    python -m py.train.from_argv path/to/argv.txt [function_name]
"""

from __future__ import annotations

import os
import sys
from dataclasses import fields, is_dataclass
from pathlib import Path

# Allow construction of FinetuneConfig without a real PSI_HOME; canonical
# pretrained paths get suppressed from the diff so the real PSI_HOME of the
# machine that runs the emitted task function is what's used.
os.environ.setdefault("PSI_HOME", "/_PSI_HOME_NOT_SET_")

from py.train.config import FinetuneConfig


SCALAR_FLAGS: dict[str, tuple[str | None, str, type]] = {
    "--seed":                                  (None,    "seed",                          int),
    "--exp":                                   (None,    "experiment",                    str),

    "--train.name":                            ("train", "name",                          str),
    "--train.data_parallel":                   ("train", "data_parallel",                 str),
    "--train.mixed_precision":                 ("train", "mixed_precision",               str),
    "--train.train_batch_size":                ("train", "train_batch_size",              int),
    "--train.max_checkpoints_to_keep":         ("train", "max_checkpoints_to_keep",       int),
    "--train.gradient_accumulation_steps":     ("train", "gradient_accumulation_steps",   int),
    "--train.learning_rate":                   ("train", "learning_rate",                 float),
    "--train.max_training_steps":              ("train", "max_training_steps",            int),
    "--train.warmup_ratio":                    ("train", "warmup_ratio",                  str),
    "--train.warmup_steps":                    ("train", "warmup_steps",                  int),
    "--train.checkpointing_steps":             ("train", "checkpointing_steps",           int),
    "--train.validation_steps":                ("train", "validation_steps",              int),
    "--train.val_num_batches":                 ("train", "val_num_batches",               int),
    "--train.max_grad_norm":                   ("train", "max_grad_norm",                 float),
    "--train.lr_scheduler_type":               ("train", "lr_scheduler_type",             str),
    "--train.lr_scheduler_kwargs.weight_decay":("train", "weight_decay",                  float),

    "--log.report_to":                         ("log",   "report_to",                     str),

    "--data.root_dir":                         ("data",  "root_dir",                      str),
    "--data.train_repo_ids":                   ("data",  "train_repo_ids",                str),
    "--data.transform.repack.pad-action-dim":  ("data",  "repack_pad_action_dim",         int),
    "--data.transform.repack.pad-state-dim":   ("data",  "repack_pad_state_dim",          int),
    "--data.transform.field.stat-path":        ("data",  "stat_path",                     str),
    "--data.transform.field.stat-action-key":  ("data",  "stat_action_key",               str),
    "--data.transform.field.stat-state-key":   ("data",  "stat_state_key",                str),
    "--data.transform.field.action_norm_type": ("data",  "action_norm_type",              str),
    "--data.transform.field.pad-action-dim":   ("data",  "pad_action_dim",                int),
    "--data.transform.field.pad-state-dim":    ("data",  "pad_state_dim",                 int),

    "--model.model_name_or_path":              ("model", "model_name_or_path",            str),
    "--model.pretrained_action_header_path":   ("model", "pretrained_action_header_path", str),
    "--model.noise-scheduler":                 ("model", "noise_scheduler",               str),
    "--model.train-diffusion-steps":           ("model", "train_diffusion_steps",         int),
    "--model.n_conditions":                    ("model", "n_conditions",                  int),
    "--model.action-chunk-size":               ("model", "action_chunk_size",             int),
    "--model.action-dim":                      ("model", "action_dim",                    int),
    "--model.action-exec-horizon":             ("model", "action_exec_horizon",           int),
    "--model.observation-horizon":             ("model", "observation_horizon",           int),
    "--model.odim":                            ("model", "odim",                          int),
    "--model.view_feature_dim":                ("model", "view_feature_dim",              int),
    "--model.max-delay":                       ("model", "max_delay",                     int),
}

BOOLEAN_FLAGS: dict[str, tuple[str, str, bool]] = {
    "--data.transform.field.use-norm-mask":    ("data",  "use_norm_mask",   True),
    "--data.transform.field.no-use-norm-mask": ("data",  "use_norm_mask",   False),
    "--data.transform.field.normalize-state":  ("data",  "normalize_state", True),
    "--data.transform.field.no-normalize-state":("data", "normalize_state", False),
    "--data.transform.model.img-aug":          ("data",  "img_aug",         True),
    "--data.transform.model.no-img-aug":       ("data",  "img_aug",         False),
    "--model.tune-vlm":                        ("model", "tune_vlm",        True),
    "--model.no-tune-vlm":                     ("model", "tune_vlm",        False),
    "--model.use_film":                        ("model", "use_film",        True),
    "--model.no-use_film":                     ("model", "use_film",        False),
    "--model.combined_temb":                   ("model", "combined_temb",   True),
    "--model.no-combined_temb":                ("model", "combined_temb",   False),
    "--model.rtc":                             ("model", "rtc",             True),
    "--model.no-rtc":                          ("model", "rtc",             False),
}

TUPLE_FLAGS: dict[str, tuple[str, str, type, int]] = {
    "--train.lr_scheduler_kwargs.betas":       ("train", "betas",            float, 2),
    "--data.transform.model.resize.size":      ("data",  "resize_size",      int,   2),
    "--data.transform.model.center_crop.size": ("data",  "center_crop_size", int,   2),
}

# Canonical pretrained checkpoint basenames. When argv.txt points at one of
# these (under whatever PSI_HOME Sicheng's box used), we suppress the diff so
# the local PSI_HOME prefix wins on whichever machine reruns the task.
CANONICAL_PRETRAINED_BASENAMES = {
    "pre.fast.1by1.2601091803.ckpt.ego200k.he30k",
    "postpre.1by1.pad36.2601131206.ckpt.he30k",
}


def _coerce(raw: str, t: type):
    if t is int:
        return int(raw)
    if t is float:
        return float(raw)
    return raw


def parse_argv_file(path: Path) -> FinetuneConfig:
    cfg = FinetuneConfig(experiment="_unset_")

    for raw_line in path.read_text().splitlines():
        line = raw_line.strip()
        if not line or not line.startswith("--"):
            continue

        if "=" in line:
            flag, _, value_str = line.partition("=")
            value_tokens: list[str] = [value_str]
        else:
            tokens = line.split()
            flag, value_tokens = tokens[0], tokens[1:]

        if flag in BOOLEAN_FLAGS:
            section, fname, val = BOOLEAN_FLAGS[flag]
            setattr(getattr(cfg, section), fname, val)
        elif flag in TUPLE_FLAGS:
            section, fname, t, n = TUPLE_FLAGS[flag]
            if len(value_tokens) != n:
                raise ValueError(f"{flag} expected {n} values, got {value_tokens}")
            setattr(getattr(cfg, section), fname, tuple(_coerce(v, t) for v in value_tokens))
        elif flag in SCALAR_FLAGS:
            section, fname, t = SCALAR_FLAGS[flag]
            if len(value_tokens) != 1:
                raise ValueError(f"{flag} expected 1 value, got {value_tokens}")
            target = cfg if section is None else getattr(cfg, section)
            setattr(target, fname, _coerce(value_tokens[0], t))
        else:
            raise ValueError(f"Unknown flag in argv.txt: {flag}")

    return cfg


def emit_factory(name: str, parsed: FinetuneConfig) -> str:
    base = FinetuneConfig(experiment=parsed.experiment)
    body = [
        f"def {name}() -> FinetuneConfig:",
        f"    cfg = FinetuneConfig(experiment={parsed.experiment!r})",
    ]

    def emit_diff(section_name: str | None, parsed_obj, base_obj) -> None:
        for f in fields(parsed_obj):
            new = getattr(parsed_obj, f.name)
            old = getattr(base_obj, f.name)
            # nested dataclasses are walked separately per-section below
            if is_dataclass(new):
                continue
            if f.name in ("model_name_or_path", "pretrained_action_header_path"):
                if Path(new).name in CANONICAL_PRETRAINED_BASENAMES:
                    continue
            if f.name == "experiment":
                continue  # already in constructor call
            if new != old:
                prefix = f"cfg.{section_name}." if section_name else "cfg."
                body.append(f"    {prefix}{f.name} = {new!r}")

    emit_diff(None, parsed, base)
    for section in ("train", "data", "model", "log"):
        emit_diff(section, getattr(parsed, section), getattr(base, section))

    body.append("    return cfg")
    return "\n".join(body)


def main() -> int:
    if len(sys.argv) < 2:
        print("Usage: python -m py.train.from_argv <argv.txt> [function_name]")
        return 2

    argv_path = Path(sys.argv[1])
    if not argv_path.is_file():
        print(f"argv.txt not found: {argv_path}")
        return 2

    func_name = sys.argv[2] if len(sys.argv) > 2 else "recovered_task"
    cfg = parse_argv_file(argv_path)
    print(emit_factory(func_name, cfg))
    return 0


if __name__ == "__main__":
    sys.exit(main())
