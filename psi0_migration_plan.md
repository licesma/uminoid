# Psi-0 fork migration plan

Replace the `Humanoid-Exo-Learning` fork with (a) upstream Psi-0 — clean, possibly with a thin fork carrying two bugfixes — and (b) all exo-specific code in this repo (`uminoid_exo_interface`). Then validate the new setup reproduces existing finetune results before retiring the old fork.

---

## Current state

`third_party/Humanoid-Exo-Learning` is a fork of `third_party/Psi0` containing:

- **Training-load-bearing changes to Psi-0** (~60 lines, 5 files in `src/psi/`) — see "New Psi-0 fork" below.
- **Vendored H-RDT model** (~50k lines under `src/h_rdt/`) — unused by our finetune path.
- **Exo-specific application code**: data converter, deploy server, camera server, training launcher, open-loop eval, docs. Not Psi-0 code; lives in the fork for historical convenience.
- **Training recipes** (`scripts/train/psi0/0526/`, `0528/`, `finetune-pick-*.sh`) — bash wrappers with ~50 inlined `--flag=value` args each, ~95% duplicated text.

---

## Plan

### 1. New Psi-0 fork (`Psi0-uminoid` or similar)

Branched from upstream Psi-0 `main`. Contents:

- **Bugfix A — relaxed normalization ill_mask** (`src/psi/config/transform.py`).
  Replace exact `== 0` test with relative tolerance so zero-padded action/state dims (torso 28:32, base 32:36 in our 36-dim layout) don't divide by floating-point near-zero.
- **Bugfix B — action_dim mismatch handling in pretrained-action-header loader** (`src/psi/trainers/finetune.py`).
  Allow partial load when `action_dim` differs, not only when `action_chunk_size` differs.
- **Optional polish**: `Psi0Model.from_safetensors(path, ...)` classmethod (deploy convenience, not training); Qwen3-VL FA2 dtype-on-subconfigs fix.
- **Task config classes** registered with the existing draccus/tyro entry point — one per task family (e.g. `finetune_marker_pen_config`, `finetune_pick_palm_config`). Each subclasses `finetune_real_psi0_config` and sets task-specific defaults (dataset id, step count, grad-accum). This is what the bash launchers should look like once configs exist:

  ```bash
  torchrun scripts/train.py finetune_marker_pen_config --exp=$exp
  ```

- **Nothing else.** No H-RDT, no exo converter, no deploy server, no camera server, no docs about the robot. Diff vs upstream stays ~150 lines.

Open PRs upstream for Bugfix A and B in parallel. Once merged, the fork collapses to just the task config classes — or disappears entirely if upstream accepts those too.

### 2. `uminoid_exo_interface` (this repo) — what moves here

Everything that's exo-specific application code, currently in `Humanoid-Exo-Learning`:

- **Data converter** — `scripts/data/exoskeleton_to_psi0_lerobot.py` → `python/<somewhere>/exo_to_lerobot.py`. Produces `lerobot/<task>/{data, videos, meta}` against the 36-dim layout. Keep the Dex3-1 native joint angle ordering and the `action[t] = state[t+1]` fallback.
- **Training launcher** — replaces `scripts/train/psi0/0526/`, `0528/`, `finetune-pick-*.sh`. One canonical `train.sh` per task family that invokes the new fork's registered config and overrides only `exp=` / `--train.max_training_steps=`. Dated subdirectories (`0526/`, `0528/`) go away; the durable record of a run already lives in `.runs/finetune/<exp>/{argv.txt, run_config.json}`.
- **Deploy server** — `serve_psi0-rtc.sh`, `psi-inference_rtc.py`, the `/reset` endpoint, and the holder/trigger scripts. These import Psi-0 as a library; they don't modify it. Lives under `python/deploy/` or similar.
- **Camera server** — `real/teleop/image_server/realsense_server.py`. Move here, keep the hard-coded `tcp://192.168.123.164:5556` (or factor to config).
- **Open-loop eval tooling** (commit `1aebc7b` in the old fork) — needed for the comparison validation below. Lives here.
- **Documentation** — port the parts of `uminoid.md` and `replay.md` that describe *our* pipeline (data → train → deploy) into this repo's docs. Drop the parts that document Psi-0 internals.

### 3. Deprecating `Humanoid-Exo-Learning`

In order:

1. Stand up the new Psi-0 fork with the two bugfixes + at least one task config class (start with `finetune_marker_pen_config`).
2. Move the exo converter, training launcher, and deploy/camera servers into this repo. They should run end-to-end against the new fork.
3. Replace the `third_party/Humanoid-Exo-Learning` submodule with `third_party/Psi0-uminoid` (or upstream `Psi0` if the bugfix PRs land first).
4. Run the validation plan below. Do not delete the old fork submodule until validation passes.
5. Once validated, remove `third_party/Humanoid-Exo-Learning` from `.gitmodules` and from disk. Keep a tag/branch in the old fork for archival.

---

## Validation plan

Before retiring the old fork, prove the new setup reproduces existing results. Three levels, cheapest first:

1. **Training curves** — re-train `pick_marker_pen_place_box_0525` on the clean fork with identical seed (`292285`), identical LeRobot dataset, identical `meta/stats_psi0.json`, identical hyperparameters. Loss and val MSE should overlay the old run within noise.
2. **Open-loop eval** — both checkpoints predict on the same held-out episodes; compare action chunks. Differences at bf16 noise floor (~1e-3).
3. **Closed-loop on the robot** — N trials each, measure success rate. Final gate.

(1) + (2) is enough to greenlight the migration. (3) is required before deleting the old fork submodule.

**Critical pre-step**: the LeRobot dataset and `stats_psi0.json` must be byte-identical between the two runs, or you're comparing data prep, not models. Reuse Sicheng's existing `lerobot/` directory for the comparison; only validate the new converter separately, after the model comparison passes.

---

## Out of scope (do later if needed)

- H-RDT model integration — leave for whenever H-RDT actually gets used.
- Deploy-server cleanup beyond a straight port (the `psi_serve_rtc-dexmate.py` variants, simulator entry points, etc.).
- Upstreaming the task config classes — only worth doing if upstream wants per-lab configs in their repo.
