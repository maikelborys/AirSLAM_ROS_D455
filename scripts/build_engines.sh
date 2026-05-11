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

PRECISION_FLAG="${PRECISION_FLAG:---noTF32}"   # default: pure FP32 (matches paper precision)
WORKSPACE_MB="${WORKSPACE_MB:-2048}"
# OptLevel 5 picks more accurate kernels (vs default 3). The numerical_diff
# scripts in this directory show this collapses LightGlue drift vs the
# onnxruntime reference from L2=29.81 -> 0.06 (466x improvement).
OPT_LEVEL="${OPT_LEVEL:-5}"

cd "${OUTPUT_DIR}"

# Selectable subset: by default rebuild every engine. Set BUILD_ENGINES=xfeat
# (space-separated list) to limit to a few. Useful while iterating on Phase 1
# of the XFeat port without rebuilding the SuperPoint+LightGlue pair every time.
BUILD_ENGINES="${BUILD_ENGINES:-superpoint xfeat lightglue superglue plnet_s0 plnet_s1}"

want() { [[ " ${BUILD_ENGINES} " == *" $1 "* ]]; }

# Ensure the patched ONNX files exist for every engine we plan to build.
declare -A engine_onnx=(
  [superpoint]=superpoint_v1_sim_int32
  [xfeat]=xfeat
  [lightglue]=superpoint_lightglue
  [superglue]=superglue_outdoor_sim_int32
  [plnet_s0]=plnet_s0
  [plnet_s1]=plnet_s1
)
for key in ${BUILD_ENGINES}; do
  src="${engine_onnx[$key]:-}"
  if [[ -z "${src}" ]]; then
    echo "ERROR: unknown engine '${key}' — valid keys: ${!engine_onnx[*]}" >&2
    exit 1
  fi
  if [[ ! -f "${src}_trt10.onnx" ]]; then
    echo "ERROR: ${src}_trt10.onnx missing — run scripts/patch_onnx_for_trt10.py first." >&2
    exit 1
  fi
done

run_trtexec() {
  local label="$1"; shift
  echo "==> ${label}"
  trtexec "$@" --memPoolSize=workspace:${WORKSPACE_MB} \
    --builderOptimizationLevel=${OPT_LEVEL} ${PRECISION_FLAG}
}

if want superpoint; then
run_trtexec "SuperPoint v1" \
  --onnx=superpoint_v1_sim_int32_trt10.onnx \
  --saveEngine=superpoint_v1_sim_int32.engine \
  --minShapes=input:1x1x100x100 \
  --optShapes=input:1x1x500x500 \
  --maxShapes=input:1x1x1500x1500
fi

# XFeat (Verlab accelerated_features) — single input, three outputs (feats,
# keypts, rel). Same dynamic-shape contract as SuperPoint so the engine works
# across EuRoC (480x752) and D455 (480x848 / 720x1280) without rebuilding.
if want xfeat; then
run_trtexec "XFeat (FCN forward — 64-dim dense descriptors)" \
  --onnx=xfeat_trt10.onnx \
  --saveEngine=xfeat.engine \
  --minShapes=image:1x1x100x100 \
  --optShapes=image:1x1x480x752 \
  --maxShapes=image:1x1x1500x1500
fi

if want lightglue; then
run_trtexec "SuperPoint+LightGlue (fused)" \
  --onnx=superpoint_lightglue_trt10.onnx \
  --saveEngine=superpoint_lightglue.engine \
  --minShapes=keypoints_0:1x1x2,keypoints_1:1x1x2,descriptors_0:1x1x256,descriptors_1:1x1x256 \
  --optShapes=keypoints_0:1x512x2,keypoints_1:1x512x2,descriptors_0:1x512x256,descriptors_1:1x512x256 \
  --maxShapes=keypoints_0:1x1024x2,keypoints_1:1x1024x2,descriptors_0:1x1024x256,descriptors_1:1x1024x256
fi

if want superglue; then
run_trtexec "SuperGlue (outdoor)" \
  --onnx=superglue_outdoor_sim_int32_trt10.onnx \
  --saveEngine=superglue_outdoor_sim_int32.engine \
  --minShapes=keypoints_0:1x1x2,scores_0:1x1,descriptors_0:1x256x1,keypoints_1:1x1x2,scores_1:1x1,descriptors_1:1x256x1 \
  --optShapes=keypoints_0:1x256x2,scores_0:1x256,descriptors_0:1x256x256,keypoints_1:1x256x2,scores_1:1x256,descriptors_1:1x256x256 \
  --maxShapes=keypoints_0:1x512x2,scores_0:1x512,descriptors_0:1x256x512,keypoints_1:1x512x2,scores_1:1x512,descriptors_1:1x256x512
fi

if want plnet_s0; then
run_trtexec "PLNet stage 0 (point + line backbone)" \
  --onnx=plnet_s0_trt10.onnx \
  --saveEngine=plnet_s0.engine \
  --minShapes=input:1x1x100x100 \
  --optShapes=input:1x1x512x512 \
  --maxShapes=input:1x1x1500x1500
fi

# PLNet stage 1: many dynamic inputs (juncs_pred, lines_pred, ...). The shape
# bounds below match what AirSLAM sets at runtime in src/plnet.cpp.
# Note: max for loi_features* is capped at 128 (not 512) — the runtime never
# resizes above 128, and 512 inflates the engine to >2 GB.
if want plnet_s1; then
run_trtexec "PLNet stage 1 (line wireframe head)" \
  --onnx=plnet_s1_trt10.onnx \
  --saveEngine=plnet_s1.engine \
  --minShapes=juncs_pred:1x2,lines_pred:1x4,idx_lines_for_junctions:1x2,inverse:1x1,iskeep_index:1x1,loi_features:1x16x16x16,loi_features_thin:1x4x16x16,loi_features_aux:1x4x16x16 \
  --optShapes=juncs_pred:250x2,lines_pred:20000x4,idx_lines_for_junctions:20000x2,inverse:20000x1,iskeep_index:20000x1,loi_features:1x128x128x128,loi_features_thin:1x4x128x128,loi_features_aux:1x4x128x128 \
  --maxShapes=juncs_pred:300x2,lines_pred:50000x4,idx_lines_for_junctions:50000x2,inverse:50000x1,iskeep_index:50000x1,loi_features:1x128x128x128,loi_features_thin:1x4x128x128,loi_features_aux:1x4x128x128
fi

echo
echo "Done. Engines:"
ls -lh "${OUTPUT_DIR}"/*.engine
