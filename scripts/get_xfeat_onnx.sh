#!/usr/bin/env bash
# Reproducibly produce output/xfeat.onnx by:
#   1. Cloning verlab/accelerated_features at a pinned commit/tag.
#   2. Setting up a throw-away venv with pinned torch + onnx + onnxruntime.
#   3. Running export_xfeat_onnx.py.
#
# Idempotent: if output/xfeat.onnx already exists, bails out cleanly.
# Pass --force to re-export.
#
# Usage:
#   scripts/get_xfeat_onnx.sh [--force] [--height 480] [--width 752]
#
# Output: output/xfeat.onnx  (consumed by scripts/patch_onnx_for_trt10.py
#         and then by scripts/build_engines.sh).

set -euo pipefail

# `~/libtorch/lib` (used by OKVIS2-X / cerebro_robot_sim) on LD_LIBRARY_PATH
# silently overrides the venv's torch and crashes Python with
# "undefined symbol: _PyCode_SetExtra". Drop it for this script only.
LD_LIBRARY_PATH="$(echo "${LD_LIBRARY_PATH:-}" | tr ':' '\n' \
  | grep -vE '/libtorch/lib$|/libtorch$' | tr '\n' ':' | sed 's/:$//')"
export LD_LIBRARY_PATH

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
OUTPUT_DIR="${REPO_ROOT}/output"
WORK_DIR="${XFEAT_WORK_DIR:-${HOME}/.cache/airslam_xfeat}"
UPSTREAM_DIR="${WORK_DIR}/accelerated_features"
VENV_DIR="${WORK_DIR}/export_venv"

# Pinned commit / tag of verlab/accelerated_features (v0.1, released 2024-06-22).
PINNED_COMMIT="${XFEAT_PINNED_COMMIT:-v0.1}"

# Pinned export deps. PyTorch 2.4 is the last 2.x with CUDA 12 wheels still
# on PyPI; ONNX 1.17 supports opset 17 used by export_xfeat_onnx.py.
TORCH_VERSION="${XFEAT_TORCH_VERSION:-2.4.0}"
ONNX_VERSION="${XFEAT_ONNX_VERSION:-1.17.0}"
ORT_VERSION="${XFEAT_ORT_VERSION:-1.19.0}"

FORCE=0
HEIGHT=480
WIDTH=752
while [[ $# -gt 0 ]]; do
  case "$1" in
    --force) FORCE=1; shift ;;
    --height) HEIGHT="$2"; shift 2 ;;
    --width) WIDTH="$2"; shift 2 ;;
    -h|--help)
      sed -n '2,/^set -euo/p' "$0" | sed 's/^# *//'
      exit 0
      ;;
    *) echo "Unknown arg: $1" >&2; exit 2 ;;
  esac
done

mkdir -p "${OUTPUT_DIR}" "${WORK_DIR}"

if [[ -f "${OUTPUT_DIR}/xfeat.onnx" && "${FORCE}" -eq 0 ]]; then
  echo "✓ ${OUTPUT_DIR}/xfeat.onnx already exists (pass --force to re-export)"
  echo "  $(stat -c '%s bytes' "${OUTPUT_DIR}/xfeat.onnx") · sha256:"
  sha256sum "${OUTPUT_DIR}/xfeat.onnx" | awk '{print "  " $1}'
  exit 0
fi

echo "→ Working directory: ${WORK_DIR}"

# Step 1 — clone or update upstream at the pinned commit.
if [[ ! -d "${UPSTREAM_DIR}" ]]; then
  git clone https://github.com/verlab/accelerated_features.git "${UPSTREAM_DIR}"
fi
cd "${UPSTREAM_DIR}"
git fetch --tags --quiet
git checkout "${PINNED_COMMIT}"
echo "✓ accelerated_features @ $(git rev-parse --short HEAD)"

# Step 2 — temp venv. Prefer `uv` (no python3-venv apt dep), fall back to
# stdlib venv if uv is missing.
if command -v uv >/dev/null 2>&1; then
  if [[ ! -d "${VENV_DIR}" ]]; then
    uv venv --python 3.12 "${VENV_DIR}"
  fi
  # shellcheck disable=SC1091
  source "${VENV_DIR}/bin/activate"
  uv pip install \
    "torch==${TORCH_VERSION}" \
    "torchvision" \
    "onnx==${ONNX_VERSION}" \
    "onnxruntime==${ORT_VERSION}" \
    "numpy<2.0" \
    "tqdm" \
    "opencv-python-headless" \
    --index-strategy unsafe-best-match \
    --index https://download.pytorch.org/whl/cpu \
    --index https://pypi.org/simple
else
  if [[ ! -d "${VENV_DIR}" ]]; then
    python3 -m venv "${VENV_DIR}"
  fi
  # shellcheck disable=SC1091
  source "${VENV_DIR}/bin/activate"
  python -m pip install --upgrade pip wheel >/dev/null
  pip install \
    "torch==${TORCH_VERSION}" \
    "torchvision" \
    "onnx==${ONNX_VERSION}" \
    "onnxruntime==${ORT_VERSION}" \
    "numpy<2.0" \
    "tqdm" \
    "opencv-python-headless" \
    --index-url https://download.pytorch.org/whl/cpu \
    --extra-index-url https://pypi.org/simple
fi
echo "✓ export venv ready"

# Step 3 — export.
cd "${REPO_ROOT}"
python "${SCRIPT_DIR}/export_xfeat_onnx.py" \
  --xfeat-repo "${UPSTREAM_DIR}" \
  --output "${OUTPUT_DIR}/xfeat.onnx" \
  --height "${HEIGHT}" --width "${WIDTH}"

echo
echo "✓ Wrote ${OUTPUT_DIR}/xfeat.onnx"
echo "  Next: python scripts/patch_onnx_for_trt10.py ${OUTPUT_DIR}/xfeat.onnx"
echo "        then  bash scripts/build_engines.sh"
