# Stage 4 — Dataset-level statistics

`scripts/data/calc_modality_stats.py --work-dir … --task <task>`.

Reads every parquet produced by [[raw_to_lerobot]], aggregates per-feature
statistics, and writes `meta/stats.json`.

Then a manual copy:

```bash
cp meta/stats.json meta/stats_psi0.json
```

(Ψ₀ training looks for `stats_psi0.json` by convention; today the two files are
identical.)

A patch step exists for a known LeRobot metadata bug:

```bash
python scripts/data/patch_lerobot_meta.py $PSI_HOME/data/real/<task>
```

Next: [[finetune]].
