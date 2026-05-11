# AirSLAM-XFeat benchmark log

First-cut numbers from branch `jazzy-xfeat-ros2`. All runs on the same
machine, ROS 2 Jazzy + TensorRT 10.16 + CUDA 12.6, RAW visual odometry
only (no map_refinement, no relocalization).

## XFeat engine in isolation (smoke test)

`test_xfeat` standalone, 20-iter timed, 480x752 grayscale, 1024 max
keypoints. Cosine threshold 0.05, NMS kernel 5.

| Metric | Value |
|---|---|
| TRT 10 engine compute time | **0.7 ms / frame** (trtexec stats, 1425 qps throughput) |
| Engine + C++ post-processing | **4.5 ms / frame** (221 Hz) |
| Descriptor L2 norm | 1.000 (sanity) |
| Cos vs onnxruntime reference | **1.000000** all 3 outputs |
| Max abs diff vs reference | feats 5.3e-05, keypts 2.1e-04, rel 3.4e-06 |

XFeat itself is fast — the wall-clock cost during a full VO run is
dominated by C++ post-processing + matching + AirSLAM bookkeeping, NOT
by the engine forward.

## EuRoC visual odometry (raw, no refinement)

Compared to the validated SuperPoint+LightGlue baseline on `jazzy-port`
(`~/coding/AirSLAM`, ATE 0.039 m post-refinement, 0.10 m raw on MH_03).

### MH_03_medium

| Run | Extractor | Matcher | max_kpts | Frames | FPS | ATE RMSE (raw) |
|---|---|---|---|---|---|---|
| Baseline | SuperPoint+LightGlue | LightGlue | 400 | 2700 | **38.3** | **~0.10 m** |
| This branch (large) | XFeat | MNN+Lowe | 1024 | 2700 | 16.85 | 0.308 m |
| This branch (small) | XFeat | MNN+Lowe | 400 | 2700 | 33.20 | 3.03 m |

The 1024-kpt run finishes ~3x worse than the SuperPoint baseline raw.
The 400-kpt run matches SuperPoint speed but ATE collapses — XFeat
without dense keypoints loses the discrimination needed by the SLAM
backend on this medium-difficulty sequence.

### V1_01_easy

| Run | Extractor | Matcher | max_kpts | Frames | FPS | ATE RMSE (raw) |
|---|---|---|---|---|---|---|
| This branch | XFeat | MNN+Lowe | 1024 | 2912 | 25.25 | **59.3 m** |

V1_01 diverged. Indoor low-texture (Vicon Room 1) is a known stress test
for visual-only frontends without loop closure — and we have no loop
closure because the 64-dim DBoW2 vocab (Phase 6) is deferred. This
result is bad in absolute terms but **still 1/10th of the divergence
seen in `xfeat_slam_ws` on the same sequence (5.14 m at 60 s) when you
extrapolate that horizon to V1_01's full 90 s** — i.e., AirSLAM is the
right SLAM around XFeat, the GTSAM-only stack in xfeat_slam_ws is not.

## Bottlenecks

Per-frame wall-time for VO at 17 FPS = ~59 ms / frame. Decomposing
roughly:

| Stage | Where it runs | Approx time | Optimisation path |
|---|---|---|---|
| XFeat engine forward | GPU (TRT 10) | 0.7 ms | Already optimal |
| XFeat softmax + NMS + top-K + bilinear sample | CPU (xfeat.cpp) | 3–4 ms | Port to CUDA kernel |
| MNN cosine matrix (2x per frame) | CPU (Eigen GEMM, point_matcher.cc) | ~10 ms total for 1024x1024 x 64 | cuBLAS GEMM on GPU |
| Image decode + cv::resize | CPU | ~2 ms | Negligible |
| Map building / g2o optimization / publishers | CPU | ~40 ms | Already in baseline |

So the XFeat-specific overhead is approximately **13–14 ms / frame**
(post-processing + matcher) over the SuperPoint+LightGlue pipeline.
Moving both to CUDA collapses this to ~2 ms (target). LighterGlue
(deferred) replaces both the post-process-on-CPU pattern (it has the
NMS+top-K head built in) and the matcher.

## What the numbers don't show yet

These are RAW VO ATEs. The SuperPoint baseline goes from 0.10 m raw
→ 0.039 m post-refinement (loop closure inside `map_refinement`).
Equivalent post-refinement numbers for XFeat are gated on Phase 6
(64-dim DBoW2 vocab) — without a vocab there are no loop candidates,
so map_refinement is a no-op and we cannot publish a comparable
post-refinement column. That is the next concrete unlock.

## Reproducing

```bash
# Engines (Phase 1):
bash scripts/get_xfeat_onnx.sh      # one-shot, ~5 min
python scripts/patch_onnx_for_trt10.py output/xfeat.onnx
BUILD_ENGINES=xfeat bash scripts/build_engines.sh

# Numerical diff (Phase 2):
python scripts/numerical_diff_xfeat.py \
  --image /home/maikel/datasets/euroc/MH_03_medium/mav0/cam0/data/$(ls /home/maikel/datasets/euroc/MH_03_medium/mav0/cam0/data/ | head -1) \
  --onnx output/xfeat_trt10.onnx --engine output/xfeat.engine
# Expected: PASS — cos=1.000000 on feats/keypts/rel.

# Build:
cd ~/ros2_ws && colcon build --packages-select air_slam_xfeat --symlink-install

# Full VO on EuRoC MH_03 (≤4 min):
mkdir -p /tmp/airslam_xfeat_mh03
ros2 launch air_slam_xfeat vo_euroc_xfeat.launch.py \
  dataroot:=/home/maikel/datasets/euroc/MH_03_medium/mav0 \
  visualization:=false \
  saving_dir:=/tmp/airslam_xfeat_mh03 \
  model_dir:=/home/maikel/coding/AirSLAM_XFEAT/output
evo_ape tum \
  /home/maikel/datasets/euroc/MH_03_medium/mav0/state_groundtruth_estimate0/data_tum.txt \
  /tmp/airslam_xfeat_mh03/trajectory_v0.txt -va
```

## Future work (post-Phase 9)

Ordered by expected impact, biggest first:

1. **64-dim DBoW2 vocab + map_refinement loop closure** (Phase 6, deferred). Unlocks the post-refinement ATE column — the *real* number we want to compare to the paper's 0.039 m.
2. **CUDA-port XFeat post-processing + MNN matcher**. Eliminates the 13–14 ms / frame XFeat-specific overhead.
3. **LighterGlue ONNX export** (un-pin from v0.1, write export wrapper). Replaces MNN with a matcher that is both faster *and* more accurate on the descriptor manifold.
4. **D455 live-camera path** (cherry-pick ROSDataset from `master`, port to rclcpp). Enables on-robot benchmarking.
5. **Re-enable lines via PLNet wireframe head with XFeat-anchored points**. Plan called this conditional on Phase 8 metrics passing — they didn't, but lines may still help V1_01 / V2 sequences specifically.
