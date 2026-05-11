# CLAUDE.md — AirSLAM-XFeat (ROS 2 Jazzy + TensorRT 10 + XFeat 64-dim)

This file is the orientation map for Claude Code working inside `~/coding/AirSLAM_XFEAT/` on branch **`jazzy-xfeat-ros2`**. The full README delta is at `README_JAZZY_XFEAT.md`; the SuperPoint-era doc is preserved below.

## What this branch is

Fork-of-fork of [`maikelborys/AirSLAM_ROS_D455`](https://github.com/maikelborys/AirSLAM_ROS_D455) `jazzy-port` (the ROS 2 Jazzy + TRT 10 port that gets 0.0386 m ATE post-refinement on EuRoC MH_03). Active branch is **`jazzy-xfeat-ros2`**. Package was renamed `air_slam` → **`air_slam_xfeat`** so both pipelines (SuperPoint and XFeat) can coexist in the same `~/ros2_ws/` for A/B comparison.

This branch's mission: **swap SuperPoint (256-dim) for XFeat (Verlab `accelerated_features` v0.1, 64-dim)** in AirSLAM's point extraction path, generalise the descriptor dimension through the matcher, and benchmark on EuRoC. The SLAM math (g2o optimisation, IMU integration, keyframe management) is unchanged from `jazzy-port`.

## XFeat-mode hard rules (additional to the inherited ones below)

X1. **XFeat ONNX export is STATIC SHAPE only**. `modules/model.py:135` calls an InstanceNorm path that traces through `Unfold`; under dynamic axes `torch.onnx.export` raises `Unsupported: ONNX export of operator Unfold, input size not accessible`. The engine therefore expects `1x1x480x752` exactly. AirSLAM resizes every frame to that size before `XFeat::infer()` and scales keypoints back. If you change `xfeat_input_height` / `xfeat_input_width` in the YAML, you MUST re-export the ONNX with matching `--height` / `--width` and rebuild the engine.

X2. **The XFeat class outputs a 67-row Eigen matrix** (3 + 64) — not the 259-row matrix the rest of AirSLAM uses. `feature_detector.cc::DetectXFeat` pads the 64-dim descriptor up to 256-dim with **zeros** so the existing `Eigen::Matrix<float, 259, Dynamic>` plumbing keeps working. **Do not consume rows 67..258 in XFeat mode** — they are pad zeros and will give garbage cosine similarity. The MNN matcher (Phase 5) only reads rows 3..66; LighterGlue (future work) must do the same.

X3. **XFeat post-processing runs on the host (CPU)**. Softmax over 65 channels, `cv::dilate` NMS, top-K, bilinear sample of `feats` and `rel` are all in `src/xfeat.cpp`. Engine compute itself is 0.7 ms on GPU. The CPU post-processing is currently the dominant per-frame cost of the XFeat path (~3–4 ms vs SuperPoint's ~1 ms). Plan to port to CUDA when chasing FPS.

X4. **MNN matcher is the default for XFeat**. LighterGlue (the official XFeat matcher) is *not* in the pinned `accelerated_features@v0.1` we export from — it landed in a later commit. Plan deferred it (Phase 5 picked MNN), see `output/benchmarks_xfeat.md` "Future work". The MNN matcher lives inline in `src/point_matcher.cc`; descriptors are L2-normalised inside `xfeat.cpp` so cosine = dot product.

X5. **`feature_extractor` is the new dispatch key**. `read_configs.h::PLNetConfig::feature_extractor`: 0 = PLNet, 1 = SuperPoint, 2 = XFeat. The legacy `use_superpoint` bool is kept as a read-only back-compat alias. `FeatureDetector` builds **only the selected backbone** — running XFeat mode does NOT require `plnet_s0.engine` to exist.

X6. **No lines, no vocab in this branch** (yet). Lines via PLNet wireframe head are deferred (the user accepted "sin líneas y luego añadiremos si métricas buenas"); the 64-dim DBoW2 vocab is deferred (only needed for map_refinement + relocalization, not for raw VO). Both are listed in `output/benchmarks_xfeat.md` "Future work". `map_refinement` and `relocalization` will not give meaningful results in XFeat mode until the vocab lands.

## XFeat-specific critical files

| Path | Why it matters |
|---|---|
| `include/xfeat.h`, `src/xfeat.cpp` | XFeat wrapper. Mirrors `super_point.{h,cpp}`. Native 67-row output. |
| `scripts/export_xfeat_onnx.py`, `scripts/get_xfeat_onnx.sh` | Reproducible ONNX export from `verlab/accelerated_features@v0.1` via a throwaway uv venv. |
| `scripts/numerical_diff_xfeat.py` | Regression guard: cos≥0.999 vs onnxruntime CPU reference, max\|d\|≤1e-3. Run after any change to the engine, export, or post-processing. |
| `demo/test_xfeat.cpp` | Standalone smoke test — bypasses FeatureDetector, dumps top-K kpts + timings. |
| `configs/visual_odometry/vo_euroc_xfeat.yaml` | EuRoC + XFeat + MNN tunings. Mirror of `vo_euroc.yaml`. |
| `launch/visual_odometry/vo_euroc_xfeat.launch.py` | Same shape as `vo_euroc.launch.py` but `package='air_slam_xfeat'` and the XFeat yaml. |
| `output/benchmarks_xfeat.md` | First-cut ATE numbers, bottleneck analysis, future work. |

## Reproducing the bring-up

```bash
cd ~/coding/AirSLAM_XFEAT
bash scripts/get_xfeat_onnx.sh                          # ~3 min first time, idempotent
python scripts/patch_onnx_for_trt10.py output/xfeat.onnx
BUILD_ENGINES=xfeat bash scripts/build_engines.sh       # ~30 s

# Regression guards (MUST PASS):
source ~/.cache/airslam_xfeat/export_venv/bin/activate
python scripts/numerical_diff_xfeat.py \
  --image /home/maikel/datasets/euroc/MH_03_medium/mav0/cam0/data/$(ls /home/maikel/datasets/euroc/MH_03_medium/mav0/cam0/data/ | head -1) \
  --onnx output/xfeat_trt10.onnx --engine output/xfeat.engine

# Symlinked into ~/ros2_ws/src/air_slam_xfeat:
cd ~/ros2_ws && source /opt/ros/jazzy/setup.bash
colcon build --packages-select air_slam_xfeat --symlink-install

# Smoke test:
source install/setup.bash
ros2 run air_slam_xfeat test_xfeat \
  --image /home/maikel/datasets/euroc/MH_03_medium/mav0/cam0/data/$(ls /home/maikel/datasets/euroc/MH_03_medium/mav0/cam0/data/ | head -1)

# Full EuRoC VO benchmark:
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

---

# Original CLAUDE.md from jazzy-port (preserved for reference)

## What this is

Fork of [`sair-lab/AirSLAM`](https://github.com/sair-lab/AirSLAM) (TRO 2025 — point-line VIO with PLNet + LightGlue/SuperGlue + relocalization) ported to **Ubuntu 24.04 / ROS 2 Jazzy / TensorRT 10.16 / CUDA 12.6**. Active branch is `jazzy-port`. The `master` branch is the legacy ROS 1 + ros1_bridge fork for D455 — kept only for cherry-picking `ROSDataset` later. Do not mix branches.

## Hard rules

1. **Algorithm is unchanged**. This is an API-compat port. Do not redesign the SLAM (no swapping the optimizer, no rewriting feature detection). If a behaviour difference vs upstream is observed, the regression is virtually always in TRT 10 kernels or the ONNX patching, not in the SLAM math.
2. **Never let TRT 8 idioms reappear**. `getNbBindings`, `getBindingIndex`, `setBindingDimensions`, `executeV2`, `bindingIsInput` are all gone. Use the name-keyed TRT 10 API: `getNbIOTensors`, `getIOTensorName`, `setInputShape`, `getTensorShape`, `setInputTensorAddress` / `setOutputTensorAddress`, `enqueueV3(stream)`. The `BufferManager` in `3rdparty/tensorrtbuffer/include/buffers.h` is the single source of truth — every network class must call `buffers.setTensorAddresses(context_)` before `enqueueV3`.
3. **Two-library split is intentional**. `air_slam_core_lib` must stay free of `rclcpp` and `g2o::Line3D`. If you need to add a `.cc` that touches `Map` / `RosPublisher` / `g2o_optimization`, it goes into `air_slam_lib`, not core. The split keeps the network/feature path testable in isolation (see `scripts/numerical_diff_*.py`).
4. **Engines are not in git**. `*.engine` and `*_trt10.onnx` are gitignored — they are regenerable artifacts. To rebuild: `python scripts/patch_onnx_for_trt10.py output/*.onnx && bash scripts/build_engines.sh`.
5. **Default precision is FP32 + `--noTF32` + `--builderOptimizationLevel=5`**. This was empirically validated — `--fp16` regresses ATE 30% on EuRoC, default TF32 introduces drift in LightGlue (max abs diff 0.45 vs PyTorch reference). Don't change defaults without re-running the numerical diff harnesses.
6. **`utils.h` is g2o-free**. Anything that needs `g2o::Line3D` / `Line3DPtr` includes `utils_g2o.h` instead. Do not move g2o types back into `utils.h`.
7. **The 5-tuple TRT 10 inference pattern** in every `infer()`:
   1. `context_->setInputShape(name, dims)` for every dynamic-shape input.
   2. `BufferManager buffers(engine_, context_.get())` (sized from current context shapes).
   3. `process_input(buffers, ...)` populates host buffers.
   4. `buffers.setTensorAddresses(context_.get())` then `buffers.copyInputToDeviceAsync(stream_)`, `context_->enqueueV3(stream_)`, `buffers.copyOutputToHostAsync(stream_)`, `cudaStreamSynchronize(stream_)`.
   5. `process_output(buffers, ...)` reads from host buffers.

   Don't break this order.

## Workspace integration

Built from the user's standard ROS 2 workspace:

```bash
ln -s ~/coding/AirSLAM ~/ros2_ws/src/air_slam
cd ~/ros2_ws && colcon build --packages-select air_slam --symlink-install
```

Edit at `~/coding/AirSLAM/`. Build from `~/ros2_ws/`. Same pattern as glim, glim_ros2, semantic_mapping in this workspace (see `~/coding/CLAUDE.md`).

## Critical files

| Path | Why it matters |
|---|---|
| `3rdparty/tensorrtbuffer/include/buffers.h` | Linchpin of TRT 10 migration. Any future TRT version bump starts here. |
| `cmake/FindG2O.cmake` | Has the `list(FILTER ... NOTFOUND)` that lets the slim apt `libg2o-dev` work. |
| `include/utils_g2o.h` | The split-out g2o helpers. Including this in `air_slam_core_lib` sources breaks the architecture. |
| `scripts/patch_onnx_for_trt10.py` | Mandatory step before `build_engines.sh`. Inserts Int32→Int64 casts. |
| `scripts/build_engines.sh` | Single command to (re)generate all five `.engine` files with the validated flags. |
| `scripts/numerical_diff_{superpoint,lightglue}.py` | Regression guards — verify TRT engine output bit-matches onnxruntime reference. Run after any change to network classes or trtexec flags. |
| `rviz/vo_jazzy.rviz` | Single RViz2 config used by all 10 launch files. Has explicit QoS settings. |
| `CMakeLists.txt.ros1.bak` | Original ROS 1 CMakeLists. Keep for reference; do not delete. |

## Common operations

### Build + smoke test from scratch

```bash
# Patch ONNX (only needed once, or after upstream model swap):
source ~/.airslam_venv/bin/activate
python scripts/patch_onnx_for_trt10.py output/*.onnx

# Engines (~5 min):
bash scripts/build_engines.sh

# C++ build:
cd ~/ros2_ws && source /opt/ros/jazzy/setup.bash
colcon build --packages-select air_slam --symlink-install
```

### Validate the migration is still good

```bash
# Numerical diff vs PyTorch/onnxruntime — must show cosine=1.0
python scripts/numerical_diff_superpoint.py \
  --image /home/maikel/datasets/euroc/MH_03_medium/mav0/cam0/data/$(ls /home/maikel/datasets/euroc/MH_03_medium/mav0/cam0/data/ | head -1) \
  --onnx output/superpoint_v1_sim_int32_trt10.onnx \
  --engine output/superpoint_v1_sim_int32.engine

python scripts/numerical_diff_lightglue.py \
  --onnx output/superpoint_lightglue.onnx \
  --engine output/superpoint_lightglue.engine
```

### Full-pipeline validation on EuRoC MH_03_medium

```bash
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 launch air_slam vo_euroc.launch.py \
  dataroot:=/home/maikel/datasets/euroc/MH_03_medium/mav0 \
  saving_dir:=/tmp/airslam_mh03 \
  model_dir:=/home/maikel/coding/AirSLAM/output
ros2 launch air_slam mr_euroc.launch.py map_root:=/tmp/airslam_mh03 \
  model_dir:=/home/maikel/coding/AirSLAM/output
ros2 launch air_slam reloc_euroc.launch.py map_root:=/tmp/airslam_mh03 \
  dataroot:=/home/maikel/datasets/euroc/MH_03_medium/mav0/cam0/data \
  model_dir:=/home/maikel/coding/AirSLAM/output
evo_ape tum /home/maikel/datasets/euroc/MH_03_medium/mav0/state_groundtruth_estimate0/data_tum.txt \
            /tmp/airslam_mh03/trajectory_v1.txt -va
```

Pass criteria (validated 2026-05-09):
- VO ≥ 30 FPS.
- Refined ATE RMSE ≤ 0.05 m on MH_03 (achieved 0.0386 m).
- Relocalization recall = 100 % on the source images.

## Out of scope

- D455 live-camera path (cherry-pick later from the `master` branch's `ROSDataset` implementation, port to rclcpp + `message_filters`).
- Semantic layer integration (separate package — `~/coding/semantic_mapping/`).
- Nvblox fusion. Architectural intent: AirSLAM provides the global pose / relocalization, Nvblox provides the local TSDF map; the integration glue is not in this repo.
- Replacing g2o, ceres, or DBoW2.
- Re-introducing FP16 or matcher=1 (SuperGlue) as defaults — both regressed ATE in testing.

## Architecture pointer

```
~/coding/AirSLAM/                            (this repo)
   ├── include/, src/                        (C++ — split between core and ROS-aware libs)
   ├── 3rdparty/{tensorrtbuffer,DBoW2}/      (vendored, both standalone CMake)
   ├── cmake/Find{G2O,Glog}.cmake            (carry-over from upstream)
   ├── configs/, voc/, output/, rviz/        (runtime data; output/*.engine gitignored)
   ├── launch/                               (.launch.py for ROS 2)
   ├── scripts/                              (engine build + numerical diffs)
   ├── demo/{visual_odometry,map_refinement,relocalization,test_feature}.cpp  (4 executables)
   └── package.xml + CMakeLists.txt          (ament_cmake format 3)
```
