#!/usr/bin/env bash
# Regenerate AirSLAM TensorRT engines from the bundled ONNX models.
#
# The .engine files shipped with upstream were built with TensorRT 8.6 and
# WILL NOT load under TensorRT 10. After running this script there will be
# fresh .engine files in the AirSLAM `output/` directory, sized for the
# host's GPU.
#
# First pass uses FP32 (no --fp16) to isolate any TRT 10 numerical
# regressions. Once the EuRoC ATE pass is green, you can re-run with
# `--fp16` for the final speed/precision tradeoff.

set -euo pipefail

# Resolve to repo root regardless of where the script is invoked from.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
OUTPUT_DIR="${REPO_ROOT}/output"

if ! command -v trtexec >/dev/null 2>&1; then
  echo "ERROR: trtexec not found on PATH. Install TensorRT 10 or activate"\
       "the venv that exposes /usr/src/tensorrt/bin/trtexec." >&2
  exit 1
fi

PRECISION_FLAG="${PRECISION_FLAG:-}"   # set to '--fp16' for half-precision
EXTRA_FLAGS="${EXTRA_FLAGS:-}"         # e.g. '--memPoolSize=workspace:1024'

cd "${OUTPUT_DIR}"

echo "==> SuperPoint v1"
trtexec \
  --onnx=superpoint_v1_sim_int32.onnx \
  --saveEngine=superpoint_v1_sim_int32.engine \
  --minShapes=input:1x1x100x100 \
  --optShapes=input:1x1x500x500 \
  --maxShapes=input:1x1x1500x1500 \
  ${PRECISION_FLAG} ${EXTRA_FLAGS}

echo "==> SuperPoint+LightGlue (fused)"
trtexec \
  --onnx=superpoint_lightglue.onnx \
  --saveEngine=superpoint_lightglue.engine \
  ${PRECISION_FLAG} ${EXTRA_FLAGS}

echo "==> SuperGlue (outdoor)"
trtexec \
  --onnx=superglue_outdoor_sim_int32.onnx \
  --saveEngine=superglue_outdoor_sim_int32.engine \
  ${PRECISION_FLAG} ${EXTRA_FLAGS}

echo "==> SuperGlue (indoor)"
trtexec \
  --onnx=superglue_indoor_sim_int32.onnx \
  --saveEngine=superglue_indoor_sim_int32.engine \
  ${PRECISION_FLAG} ${EXTRA_FLAGS}

echo "==> PLNet stage 0 (point + line backbone)"
trtexec \
  --onnx=plnet_s0.onnx \
  --saveEngine=plnet_s0.engine \
  --minShapes=input:1x1x100x100 \
  --optShapes=input:1x1x512x512 \
  --maxShapes=input:1x1x1500x1500 \
  ${PRECISION_FLAG} ${EXTRA_FLAGS}

# PLNet stage 1 has many dynamic inputs (juncs_pred, lines_pred, ...). The
# upstream code sets profile shapes at build time (see src/plnet.cpp). If
# this step fails with the known #203 Cast/Gather issue, see README_JAZZY.md
# for the documented workaround.
echo "==> PLNet stage 1 (line wireframe head)"
trtexec \
  --onnx=plnet_s1.onnx \
  --saveEngine=plnet_s1.engine \
  --minShapes=juncs_pred:1x2,lines_pred:1x4,idx_lines_for_junctions:1x2,inverse:1x1,iskeep_index:1x1,loi_features:1x16x16x16,loi_features_thin:1x4x16x16,loi_features_aux:1x4x16x16 \
  --optShapes=juncs_pred:250x2,lines_pred:20000x4,idx_lines_for_junctions:20000x2,inverse:20000x1,iskeep_index:20000x1,loi_features:1x128x128x128,loi_features_thin:1x4x128x128,loi_features_aux:1x4x128x128 \
  --maxShapes=juncs_pred:500x2,lines_pred:50000x4,idx_lines_for_junctions:50000x2,inverse:50000x1,iskeep_index:50000x1,loi_features:1x512x512x512,loi_features_thin:1x4x512x512,loi_features_aux:1x4x512x512 \
  ${PRECISION_FLAG} ${EXTRA_FLAGS} || \
  echo "WARN: PLNet stage 1 engine build failed — see README_JAZZY.md issue #203."

echo "Done. Engines are in: ${OUTPUT_DIR}"
ls -lh "${OUTPUT_DIR}"/*.engine
