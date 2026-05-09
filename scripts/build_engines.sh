#!/usr/bin/env bash
# Regenerate AirSLAM TensorRT engines from the bundled ONNX models.
#
# The .engine files shipped with upstream were built with TensorRT 8.6 and
# WILL NOT load under TensorRT 10. After running this script there will be
# fresh .engine files in the AirSLAM `output/` directory.
#
# **Pre-requisite**: the ONNX files must first be patched for TRT 10 with
#     python scripts/patch_onnx_for_trt10.py output/*.onnx
# This produces *_trt10.onnx siblings with Int32->Int64 casts inserted on
# Concat/Mul/Add/etc. nodes that TRT 10's stricter ONNX importer rejects.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
OUTPUT_DIR="${REPO_ROOT}/output"

if ! command -v trtexec >/dev/null 2>&1; then
  echo "ERROR: trtexec not found on PATH. Install TensorRT 10 first." >&2
  exit 1
fi

PRECISION_FLAG="${PRECISION_FLAG:-}"   # set to '--fp16' for half-precision
WORKSPACE_MB="${WORKSPACE_MB:-2048}"

cd "${OUTPUT_DIR}"

# Ensure the patched ONNX files exist.
for src in superpoint_v1_sim_int32 plnet_s0 plnet_s1 superpoint_lightglue \
           superglue_outdoor_sim_int32; do
  if [[ ! -f "${src}_trt10.onnx" ]]; then
    echo "ERROR: ${src}_trt10.onnx missing — run scripts/patch_onnx_for_trt10.py first." >&2
    exit 1
  fi
done

run_trtexec() {
  local label="$1"; shift
  echo "==> ${label}"
  trtexec "$@" --memPoolSize=workspace:${WORKSPACE_MB} ${PRECISION_FLAG}
}

run_trtexec "SuperPoint v1" \
  --onnx=superpoint_v1_sim_int32_trt10.onnx \
  --saveEngine=superpoint_v1_sim_int32.engine \
  --minShapes=input:1x1x100x100 \
  --optShapes=input:1x1x500x500 \
  --maxShapes=input:1x1x1500x1500

run_trtexec "SuperPoint+LightGlue (fused)" \
  --onnx=superpoint_lightglue_trt10.onnx \
  --saveEngine=superpoint_lightglue.engine \
  --minShapes=keypoints_0:1x1x2,keypoints_1:1x1x2,descriptors_0:1x1x256,descriptors_1:1x1x256 \
  --optShapes=keypoints_0:1x512x2,keypoints_1:1x512x2,descriptors_0:1x512x256,descriptors_1:1x512x256 \
  --maxShapes=keypoints_0:1x1024x2,keypoints_1:1x1024x2,descriptors_0:1x1024x256,descriptors_1:1x1024x256

run_trtexec "SuperGlue (outdoor)" \
  --onnx=superglue_outdoor_sim_int32_trt10.onnx \
  --saveEngine=superglue_outdoor_sim_int32.engine \
  --minShapes=keypoints_0:1x1x2,scores_0:1x1,descriptors_0:1x256x1,keypoints_1:1x1x2,scores_1:1x1,descriptors_1:1x256x1 \
  --optShapes=keypoints_0:1x256x2,scores_0:1x256,descriptors_0:1x256x256,keypoints_1:1x256x2,scores_1:1x256,descriptors_1:1x256x256 \
  --maxShapes=keypoints_0:1x512x2,scores_0:1x512,descriptors_0:1x256x512,keypoints_1:1x512x2,scores_1:1x512,descriptors_1:1x256x512

run_trtexec "PLNet stage 0 (point + line backbone)" \
  --onnx=plnet_s0_trt10.onnx \
  --saveEngine=plnet_s0.engine \
  --minShapes=input:1x1x100x100 \
  --optShapes=input:1x1x512x512 \
  --maxShapes=input:1x1x1500x1500

# PLNet stage 1: many dynamic inputs (juncs_pred, lines_pred, ...). The shape
# bounds below match what AirSLAM sets at runtime in src/plnet.cpp.
# Note: max for loi_features* is capped at 128 (not 512) — the runtime never
# resizes above 128, and 512 inflates the engine to >2 GB.
run_trtexec "PLNet stage 1 (line wireframe head)" \
  --onnx=plnet_s1_trt10.onnx \
  --saveEngine=plnet_s1.engine \
  --minShapes=juncs_pred:1x2,lines_pred:1x4,idx_lines_for_junctions:1x2,inverse:1x1,iskeep_index:1x1,loi_features:1x16x16x16,loi_features_thin:1x4x16x16,loi_features_aux:1x4x16x16 \
  --optShapes=juncs_pred:250x2,lines_pred:20000x4,idx_lines_for_junctions:20000x2,inverse:20000x1,iskeep_index:20000x1,loi_features:1x128x128x128,loi_features_thin:1x4x128x128,loi_features_aux:1x4x128x128 \
  --maxShapes=juncs_pred:300x2,lines_pred:50000x4,idx_lines_for_junctions:50000x2,inverse:50000x1,iskeep_index:50000x1,loi_features:1x128x128x128,loi_features_thin:1x4x128x128,loi_features_aux:1x4x128x128

echo
echo "Done. Engines:"
ls -lh "${OUTPUT_DIR}"/*.engine
