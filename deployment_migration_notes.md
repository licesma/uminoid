# Deployment migration notes

Things from `Humanoid-Exo-Learning` that the deploy code in this repo needs to take over once we switch to upstream Psi-0.

## `Psi0Model.from_safetensors` (from `src/psi/models/psi0.py`)

The fork adds a `from_safetensors(safetensors_path, launch_config, device)` classmethod that loads a checkpoint from a direct path, bypassing the `.runs/<exp>/checkpoints/ckpt_<step>/` layout that upstream's `from_pretrained` requires.

Upstream-only `from_pretrained` signature:

```python
Psi0Model.from_pretrained(run_dir, ckpt_step, launch_config, device)
# always loads {run_dir}/checkpoints/ckpt_{ckpt_step}/model.safetensors
```

Used by the deploy server when pointing at a checkpoint that doesn't live in the canonical training output layout.

**Migration:** the deploy server code (when moved into `uminoid_exo_interface`) can either

1. Stage the checkpoint into the expected `<dir>/checkpoints/ckpt_<step>/model.safetensors` layout before calling `from_pretrained` (symlink is fine), or
2. Wrap `from_pretrained` locally — copy the body and accept a direct path, no Psi-0 modification needed.

Option 1 is simpler and means no code duplication.
