# How to train your dragon

Steps to finetune a Psi-0 model via `py/train`.

## Step 1 — Build the Psi-0 environment (`.venv-psi`)

Psi-0 is managed with `uv` (not pip) and pins Python `==3.10.*`. Do **not** reuse
the `uminoid` conda env (Python 3.11, holds the realsense/dynamixel/manus
collection stack). The real deps live in uv dependency *groups* + a separate
flash-attn build, so a plain `pip install -e` produces a broken env (no torch).
`run.py` looks for `.venv-psi` at the repo root, so build it there.

### 1a. Install uv

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
source ~/.bashrc   # put uv on PATH
```

### 1b. Check out the SIMPLE submodule (required for `uv sync`)

`Psi0/pyproject.toml` declares editable path deps into `third_party/SIMPLE`
(the lab's simulator, private repo). `uv sync` validates those paths even though
training never uses SIMPLE, so the submodule must exist on disk. Needs SSH
access to the lab's private GitHub repos.

```bash
cd ~/repos/uminoid/third_party/Psi0
GIT_LFS_SKIP_SMUDGE=1 git submodule update --init --recursive third_party/SIMPLE
```

### 1c. Create the venv and sync deps

```bash
cd ~/repos/uminoid/third_party/Psi0
uv venv ~/repos/uminoid/.venv-psi --python 3.10
source ~/repos/uminoid/.venv-psi/bin/activate
GIT_LFS_SKIP_SMUDGE=1 uv sync \
  --group serve --group viz --group psi \
  --index-strategy unsafe-best-match --active
```

### 1d. Build flash-attn against CUDA 12.8

The README compiles flash-attn from source, which needs an `nvcc` matching
torch's CUDA major (torch is `cu126`). The system `nvcc` is often 11.5 (too
old). Use a 12.x toolkit — `cuda-12.8` is present on all our machines; install
it (toolkit only, no driver) where missing:

```bash
# only if /usr/local/cuda-12.8 is missing (Ubuntu 22.04 / x86_64):
cd /tmp
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb && sudo apt-get update
sudo apt-get install -y cuda-toolkit-12-8   # NOT `cuda` / `cuda-12-8` (those pull a driver)
```

Point this shell at 12.8 and build:

```bash
export CUDA_HOME=/usr/local/cuda-12.8
export PATH=$CUDA_HOME/bin:$PATH
export LD_LIBRARY_PATH=$CUDA_HOME/lib64:$LD_LIBRARY_PATH
nvcc -V   # confirm release 12.8

cd ~/repos/uminoid/third_party/Psi0
source ~/repos/uminoid/.venv-psi/bin/activate   # MUST be active (see note)
MAX_JOBS=4 uv pip install flash_attn==2.7.4.post1 --no-build-isolation
```

> **Note — a uv venv has no `pip`.** `.venv-psi/bin/pip` does not exist, so
> `.venv-psi/bin/pip install flash_attn ...` silently installs nothing. Always
> install with `uv pip install` while `.venv-psi` is active (or pass
> `--python ~/repos/uminoid/.venv-psi/bin/python`) so the build lands in the
> right environment.

(Slow, RAM-hungry compile — run in tmux. Alternatively, the matching prebuilt
wheel is `flash_attn-2.7.4.post1+cu12torch2.7cxx11abiTRUE-cp310-cp310-linux_x86_64.whl`
from Dao-AILab releases, which skips the build entirely.)

### 1e. Verify

```bash
python -c "import psi; print(psi.__version__)"
python -c "import torch; print(torch.__version__, torch.cuda.is_available())"
python -c "import flash_attn; print(flash_attn.__version__)"
```

### 1f. Convenience alias

`.venv-psi` is a uv venv (not a conda env), so `conda activate` can't reach it.
Add an alias to activate it from anywhere:

```bash
echo "alias psi0_env='source ~/repos/uminoid/.venv-psi/bin/activate'" >> ~/.bashrc
source ~/.bashrc
```

Then just `psi0_env`. (If the prompt shows `(.venv-psi) (base)`, that's harmless —
`.venv-psi` is first on PATH so it wins. To drop the conda `(base)` tag:
`conda config --set auto_activate_base false`.)

## Step 2 — Authenticate with HuggingFace

Checkpoints and datasets are pulled from the `USC-PSI-Lab` HuggingFace org, which
requires a login. Create a **Read** token at
https://huggingface.co/settings/tokens, then (with `psi0_env` active):

```bash
hf auth login        # paste the token; saved to ~/.cache/huggingface/token
hf auth whoami       # confirm it prints your username
```

This persists per machine — do it once. If a download later returns 403, the
repo is gated: open https://huggingface.co/USC-PSI-Lab/psi-model while logged in
and request/accept access.

### wandb (training logs)

The task configs set `--log.report_to=wandb`, so training needs wandb auth.
Get your key at https://wandb.ai/authorize, then either log in once per machine:

```bash
wandb login        # paste key; saved to ~/.netrc
```

or add it to the repo-root `.env` (loaded by `run.py`, travels across machines):

```bash
# ~/repos/uminoid/.env   (.env is gitignored — safe for secrets)
WANDB_API_KEY=<your key>
WANDB_ENTITY=<your wandb team or username>   # optional
```

For a quick smoke test with no account, prepend `WANDB_MODE=offline` to the
launch command instead.

## Step 3 — Download the pretrained checkpoints

`PSI_HOME` (set in `~/repos/uminoid/.env`, default `/hfm`) is the root where
checkpoints live. `run.py` resolves the base model + action expert as
`$PSI_HOME/cache/checkpoints/psi0/...`. Download the two **Baseline** checkpoints
(the pair Sicheng's finetune used):

`scripts/data/download.py` requires a `.env` in the Psi-0 repo root, so symlink
it to the single repo-root `.env`:

```bash
ln -sf ~/repos/uminoid/.env ~/repos/uminoid/third_party/Psi0/.env
```

```bash
# create /hfm once if missing, then load PSI_HOME from .env
sudo mkdir -p /hfm && sudo chown $USER:$USER /hfm
set -a; source ~/repos/uminoid/.env; set +a

cd ~/repos/uminoid/third_party/Psi0
for d in \
  psi0/pre.fast.1by1.2601091803.ckpt.ego200k.he30k \
  psi0/postpre.1by1.pad36.2601131206.ckpt.he30k ; do
  python scripts/data/download.py \
    --repo-id=USC-PSI-Lab/psi-model \
    --remote-dir=$d \
    --local-dir=$PSI_HOME/cache/checkpoints/$d \
    --repo-type=model
done
```

The VLM backbone is multi-GB — run in tmux, ensure disk space at `$PSI_HOME`.
