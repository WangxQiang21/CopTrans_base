#!/usr/bin/env bash
set -euo pipefail
root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$root_dir"
if ! command -v uv >/dev/null; then
  echo "uv is required. Install uv, then rerun this script." >&2
  exit 2
fi
nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader
export UV_CACHE_DIR="${UV_CACHE_DIR:-/tmp/coptrans-uv-cache}"
export UV_HTTP_TIMEOUT="${UV_HTTP_TIMEOUT:-120}"
if [[ ! -x .venv-rl/bin/python ]]; then
  uv venv --python "${COPTRANS_RL_PYTHON:-python3.12}" .venv-rl
fi
uv pip install --python .venv-rl/bin/python -r rl/requirements-foundation.lock
uv pip install --python .venv-rl/bin/python 'torch==2.8.0+cu128' --index-url https://download.pytorch.org/whl/cu128
uv pip install --python .venv-rl/bin/python 'stable-baselines3==2.9.0' 'sb3-contrib==2.9.0'
uv pip install --python .venv-rl/bin/python --no-deps -e ./rl
.venv-rl/bin/python - <<'PY'
import torch
if not torch.cuda.is_available():
    raise SystemExit('CUDA is not visible. Run this script in the GPU-visible host terminal.')
x=torch.randn(256,256,device='cuda')
y=x@x.T
torch.cuda.synchronize()
assert torch.isfinite(y).all()
print('CUDA verified:', torch.__version__, torch.version.cuda, torch.cuda.get_device_name(0))
PY
