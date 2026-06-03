# Stage 5 — Fine-tune

`scripts/train/psi0/finetune-real-psi0.sh <task>` consumes
`$PSI_HOME/data/real/<task>/` (the exact layout produced by [[raw_to_lerobot]] +
[[stats]]) and produces a fine-tuned Ψ₀ checkpoint.

The training-side LeRobot loader keys off the schema declared in
`meta/info.json` (the `observation.images.egocentric` video feature plus the
flat `states` / `action` float32 sequences) — so anything we want consumed by
Ψ₀ must end up in **exactly** that shape: states 32-D, action 36-D, 30 fps,
one mp4 per episode, one parquet per episode under the chunked layout.

## Summary cheat-sheet (full pipeline)

| Stage | Producer | Output | Key shapes (G1) |
|---|---|---|---|
| 1A | `RobotDataWorker` | `color/*.jpg`, `depth/*.npy.lzma`, `robot_data.jsonl` | 640×480; states 285-D snapshot |
| 1B | `RobotTaskmaster` | `ik_data.jsonl` | sol_q 29-D, hands 7-D each |
| 2  | `DataMerger` | `data.json` | per-frame list with merged `actions` |
| 3  | `HE2LeRobotConverter` | `data/*.parquet`, `videos/*.mp4`, `meta/*` | **states 32-D, action 36-D**, 30 fps |
| 4  | `calc_modality_stats.py` | `meta/stats.json` (+ `stats_psi0.json`) | per-feature aggregate stats |
| 5  | `finetune-real-psi0.sh` | Ψ₀ checkpoint | — |
