# AirSLAM-XFeat (jazzy-xfeat-ros2 branch)

Branch deltas from [`README_JAZZY.md`](./README_JAZZY.md). Read that first
for the SuperPoint + LightGlue baseline (ATE 0.039 m post-refinement on
EuRoC MH_03). This document covers what changes when you swap SuperPoint
for **XFeat** (Verlab `accelerated_features` v0.1, 64-dim dense FCN).

## Why XFeat

- Lighter (~700 K params vs SuperPoint's ~1.3 M); the FCN forward runs
  in **0.7 ms on TRT 10** (1425 qps).
- Robust to illumination / viewpoint changes (CVPR 2024 paper claims).
- Apache-2.0 licence; ONNX-exportable from upstream's PyTorch model.

The downside: no LighterGlue at the pinned `v0.1` tag we export from, and
post-processing (softmax + NMS + bilinear sample) is heavier than
SuperPoint's. See `output/benchmarks_xfeat.md` for numbers.

## What this branch changes

| Layer | jazzy-port (SuperPoint) | jazzy-xfeat-ros2 (XFeat) |
|---|---|---|
| Package name | `air_slam` | **`air_slam_xfeat`** |
| Feature extractor | SuperPoint, 256-dim | XFeat, 64-dim |
| Engine | `superpoint_v1_sim_int32.engine` (dynamic shape) | `xfeat.engine` (STATIC 1x1x480x752) |
| ONNX export | upstream-shipped ONNX | Reproducible via `scripts/get_xfeat_onnx.sh` (clones `verlab/accelerated_features@v0.1`, builds a throwaway uv venv with `torch 2.4.0 + onnx 1.17.0`, exports) |
| Engine post-proc | None — engine emits scores+desc directly | Heavy: softmax(65) → drop dustbin → upsample 8x8 → cv::dilate NMS → top-K → bilinear sample of feats and rel, in `src/xfeat.cpp` (~3–4 ms CPU per frame) |
| Point matcher | LightGlue (256-dim TRT engine) | **MNN + Lowe** (64-dim, pure Eigen GEMM, no engine) |
| DBoW2 vocab | `voc/point_voc_L4.bin` (256-dim) | **Deferred** — Phase 6 not in this branch. Map refinement / relocalization are no-ops in XFeat mode until vocab lands. |
| Lines | PLNet wireframe head | **Disabled** (XFeat is points-only here). PLNet not built when `feature_extractor=2`. |
| Config dispatch | `use_superpoint: 0/1` (bool) | `feature_extractor: 0/1/2` (enum). Legacy `use_superpoint` still read as fallback. |

The SLAM math (g2o optimisation, IMU preintegration, keyframe selection)
is **unchanged**. Same hard rule as jazzy-port.

## Hard rules specific to this branch

See [`CLAUDE.md`](./CLAUDE.md) "XFeat-mode hard rules" (X1–X6) for the
full list. The two with the most teeth:

- **Engine is static-shape**. The XFeat ONNX export fails under dynamic
  axes because of an `Unfold` in `modules/model.py:135`. Default size is
  480 x 752 (EuRoC). To change it: re-export with `--height/--width`,
  re-patch, rebuild the engine.
- **Padded descriptors**. AirSLAM consumes `Eigen::Matrix<float, 259, N>`
  feature matrices. XFeat fills rows 0–66 (3 + 64); rows 67–258 are
  **zeros**. Any new consumer in XFeat mode must read only rows 3..66.

## Bring-up

```bash
cd ~/coding/AirSLAM_XFEAT          # already on jazzy-xfeat-ros2

# Engines + ONNX:
bash scripts/get_xfeat_onnx.sh                     # ~3 min, idempotent
python scripts/patch_onnx_for_trt10.py output/xfeat.onnx
BUILD_ENGINES=xfeat bash scripts/build_engines.sh  # ~30 s

# Numerical-diff regression guard (must pass — gate cos>=0.999, max|d|<=1e-3):
source ~/.cache/airslam_xfeat/export_venv/bin/activate
python scripts/numerical_diff_xfeat.py \
  --image /home/maikel/datasets/euroc/MH_03_medium/mav0/cam0/data/$(ls /home/maikel/datasets/euroc/MH_03_medium/mav0/cam0/data/ | head -1) \
  --onnx output/xfeat_trt10.onnx --engine output/xfeat.engine

# Build under ros2_ws (symlink ~/ros2_ws/src/air_slam_xfeat must exist):
cd ~/ros2_ws && source /opt/ros/jazzy/setup.bash
colcon build --packages-select air_slam_xfeat --symlink-install

# Sanity smoke test (bypasses FeatureDetector + MapBuilder):
source install/setup.bash
ros2 run air_slam_xfeat test_xfeat \
  --image /home/maikel/datasets/euroc/MH_03_medium/mav0/cam0/data/<your-png>
# Expected: ~1024 keypoints, ~4.5 ms / frame, descriptor norms == 1.000
```

## Running EuRoC visual odometry

```bash
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

The ROS 2 topics published while VO runs are the same as `jazzy-port`
(unchanged `RosPublisher`): `/AirSLAM/feature`, `/AirSLAM/frame_pose`,
`/AirSLAM/keyframe`, `/AirSLAM/odometry` (path), `/AirSLAM/map`, and the
`map → world` TF.

## Benchmark snapshot (raw VO, no refinement)

| Sequence | XFeat+MNN ATE | XFeat FPS | SuperPoint baseline ATE | SuperPoint FPS |
|---|---|---|---|---|
| EuRoC MH_03_medium | **0.308 m** | 16.85 | ~0.10 m raw / 0.039 m post-refinement | 38.3 |
| EuRoC V1_01_easy   | 59.3 m (diverged) | 25.3 | — | — |

See `output/benchmarks_xfeat.md` for the bottleneck breakdown and the
future-work list (vocab 64-dim, CUDA post-proc, LighterGlue, D455 live
topics).

## File map (additions only)

```
~/coding/AirSLAM_XFEAT/
  include/xfeat.h                                NEW  XFeat wrapper header
  src/xfeat.cpp                                  NEW  XFeat wrapper + post-proc
  demo/test_xfeat.cpp                            NEW  smoke test
  scripts/export_xfeat_onnx.py                   NEW  PyTorch -> ONNX
  scripts/get_xfeat_onnx.sh                      NEW  orchestrator (venv + clone + export)
  scripts/numerical_diff_xfeat.py                NEW  regression guard
  configs/visual_odometry/vo_euroc_xfeat.yaml    NEW  EuRoC XFeat config
  launch/visual_odometry/vo_euroc_xfeat.launch.py NEW launch
  output/xfeat.onnx                              NEW  exported XFeat (2.7 MB, tracked)
  output/xfeat_trt10.onnx                        gitignored (regenerable)
  output/xfeat.engine                            gitignored (regenerable, 7.7 MB)
  output/benchmarks_xfeat.md                     NEW  benchmark log
  README_JAZZY_XFEAT.md                          THIS FILE
  CLAUDE.md                                      UPDATED (XFeat-mode hard rules prepended)

  include/feature_detector.h                     UPDATED (XFeatPtr + DetectXFeat)
  src/feature_detector.cc                        UPDATED (dispatch on feature_extractor)
  include/read_configs.h                         UPDATED (FeatureExtractor enum, XFeatConfig,
                                                          PointMatcherKind enum + MNN knobs)
  src/point_matcher.cc                           UPDATED (matcher==2 MNN+Lowe branch)
  scripts/build_engines.sh                       UPDATED (BUILD_ENGINES allowlist + xfeat line)

  package.xml                                    RENAMED to air_slam_xfeat
  CMakeLists.txt                                 RENAMED project() + EXPORT
  launch/**/*.launch.py                          UPDATED 'air_slam' -> 'air_slam_xfeat'
```
