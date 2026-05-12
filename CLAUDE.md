# CLAUDE.md — AirSLAM-XFeat (ROS 2 Jazzy + TensorRT 10 + XFeat + LighterGlue)

Orientation map for Claude Code inside `~/coding/AirSLAM_XFEAT/` on branch
**`jazzy-xfeat-ros2`**. The README delta is `README_JAZZY_XFEAT.md`,
architecture diagram is `ARCHITECTURE.md`, current measured numbers are
in `STATUS.md` and `output/benchmarks_xfeat.md`. The SuperPoint-era
upstream doc is preserved at the bottom of this file.

## What this branch is

Fork-of-fork of [`maikelborys/AirSLAM_ROS_D455`](https://github.com/maikelborys/AirSLAM_ROS_D455)
`jazzy-port`. Package was renamed `air_slam` → **`air_slam_xfeat`** so both
pipelines (SuperPoint and XFeat) can coexist in `~/ros2_ws/src/` for A/B
comparison.

**Mission**: swap SuperPoint (256-dim, MagicLeap non-commercial license) for
**XFeat** (Verlab `accelerated_features`, 64-dim, Apache-2.0) +
**LighterGlue** (kornia LightGlue with XFeat weights, Apache-2.0) so the
whole stack is commercially deployable. AirSLAM's SLAM backend (g2o BA,
IMU preintegration, keyframe management, DBoW2 loop closure) is untouched.

**Status, EuRoC MH_03_medium (validated)**:

```
                            FPS (full)    Raw VO ATE    Post-refine ATE
SuperPoint+LightGlue          38.3        ~0.10 m       0.039 m  (paper)
XFeat+LighterGlue+vocab+KF    51.3 🔥      0.207 m       0.070 m   <- THIS BRANCH
                              +34% vs SP                ~1.8x SP, Apache-2.0
```

The 51.3 FPS is the headline: **faster than the SuperPoint baseline on the
same hardware** while staying Apache-2.0 license-clean. The ~1.8x ATE gap
is the price of dropping the SuperPoint license taint — XFeat was not
designed to beat SuperPoint on EuRoC benchmarks; it was designed to be
commercially usable and robust to D455-class conditions (motion blur,
viewpoint, low light).

## Hard rules — XFeat-mode (additional to inherited TRT 10 / 2-lib rules)

**X1. The XFeat ONNX export is static shape, period.**
`modules/model.py:135` calls an InstanceNorm path that traces through
`Unfold`; `torch.onnx.export` raises `Unsupported: ONNX export of operator
Unfold, input size not accessible` under dynamic axes. The engine
therefore expects exactly `1x1xHxW` where (H, W) = (xfeat_input_height,
xfeat_input_width) from the YAML — default 480x752. AirSLAM resizes every
frame to that size in `XFeat::infer()` and scales keypoints back. To
change input size: re-export the ONNX with matching `--height/--width`
and rebuild the engine.

**X2. The XFeat-mode feature matrix is 259-row with rows 67..258 zeroed.**
XFeat natively produces 67 = 3 + 64 dims (score, x, y, 64-dim descriptor).
We pad to 259 rows in `feature_detector.cc::DetectXFeat()` so the rest of
AirSLAM's 256-dim-aware plumbing keeps working. **Any new consumer must
read only rows 3..66 in XFeat mode** — rows 67..258 are pad zeros and
will give garbage cosine similarity. MNN matcher and LighterGlue both
read rows 3..66.

**X3. XFeat post-processing is on CUDA**, not on CPU.
`src/xfeat_postproc.cu` runs the 4-kernel pipeline (softmax(65) +
unfold + NMS-dilate + emit-candidates + bilinear-sample) directly on
the TRT engine's device buffers via `BufferManager::getDeviceBuffer`.
H2D copy of the dense outputs (~1.5 MiB) is skipped. Bit-identical to
the CPU fallback within float32 rounding. The CPU path is preserved as
the `postproc_state_ == nullptr` branch in `XFeat::infer` but should
not run in production.

**X4. LighterGlue is loaded as a TorchScript .pt**, not a TRT engine.
`torch.onnx.export` cannot lower the kornia/cvg LightGlue forward
(negative-index `.transpose(-1, -2)`, `.unflatten(-1, ...)`,
`.shape[-X]` ops trip the symbolic exporter). `torch.jit.trace` handles
them natively. The .pt is loaded via libtorch's `torch::jit::load`
inside `src/lighter_glue.cc`. The trace is **STATIC keypoint count N**
baked at export time — config `max_keypoints` MUST match
`scripts/export_lighterglue_torchscript.py --num-kpts`. Default is 512.

**X5. Keyframe insertion thresholds need XFeat-specific tuning.**
AirSLAM's default `keyframe.tracking_point_rate: 0.65`, `max_num_match:
80`, `tracking_parallax_rate: 0.10` were tuned for SuperPoint+LightGlue's
dense matching at kpts=400. XFeat+LighterGlue at kpts=512 matches less
densely — those defaults trip a keyframe every 2.4 frames (1130/2700 on
MH_03) instead of every 9 (SuperPoint baseline's 302/2700). The
configs/visual_odometry/vo_euroc_xfeat_lighterglue.yaml ships:
`tracking_point_rate: 0.40`, `max_num_match: 40`, `parallax_rate: 0.20`
which gives 408 keyframes and 51.3 FPS. **Don't revert these without
re-benchmarking.** See `output/benchmarks_xfeat.md` "BEST" row.

**X6. `feature_extractor` is the dispatch key.**
`PLNetConfig::feature_extractor`: 0 = PLNet (points + lines, currently
unused in XFeat mode), 1 = SuperPoint (legacy A/B), 2 = XFeat. The
legacy `use_superpoint` bool is kept as a read-only back-compat alias
inside `PLNetConfig::Load()`. `FeatureDetector` builds **only the
selected backbone** — running XFeat mode does NOT need
`plnet_s0.engine` to exist on disk.

**X7. `PointMatcherKind` is the matcher dispatch.**
`read_configs.h::PointMatcherKind`: 0 = LightGlue (SuperPoint, 256-dim,
TRT), 1 = SuperGlue (SuperPoint, 256-dim, TRT), 2 = MNN (XFeat, 64-dim,
cuBLAS on GPU), 3 = **LighterGlue** (XFeat, 64-dim, TorchScript via
libtorch). The XFeat default is 3.

**X8. The DBoW2 vocab for XFeat mode is the zero-pad trick.**
`voc/point_voc_L4_xfeat.bin` was trained over XFeat descriptors padded
to 256-dim with zeros, using the existing `FSuperpoint` (L=256) class.
The zeros contribute 0 to L2 distance, so the vocabulary is effectively
64-dim in disguise. **No Database / FSuperpoint refactor needed**, and
the existing `Database::FrameToBow` extraction
(`features.block(3, i, 256, 1)`) just works. See `demo/train_voc_xfeat.cpp`.

## XFeat-specific critical files

| Path | Why it matters |
|---|---|
| `include/xfeat.h`, `src/xfeat.cpp` | XFeat wrapper. Static-shape, TRT 10 5-step infer pattern, CUDA post-proc dispatch. |
| `include/xfeat_postproc.h`, `src/xfeat_postproc.cu` | 4 CUDA kernels: softmax+unfold, NMS-dilate, emit-candidates, sample-descriptors. |
| `include/lighter_glue.h`, `src/lighter_glue.cc` | TorchScript matcher wrapper. Hides libtorch types behind pimpl. Static N pad. |
| `scripts/export_xfeat_onnx.py`, `scripts/get_xfeat_onnx.sh` | Reproducible XFeat ONNX export from `verlab/accelerated_features@v0.1`. Static-shape. |
| `scripts/export_lighterglue_torchscript.py` | LighterGlue TorchScript export from cvg/LightGlue + xfeat-lighterglue.pt weights. Trace on CUDA. |
| `scripts/numerical_diff_xfeat.py` | Regression guard: cos≥0.999 vs onnxruntime CPU reference. |
| `demo/test_xfeat.cpp` | Standalone smoke test — bypasses FeatureDetector. 564 Hz with CUDA postproc. |
| `demo/train_voc_xfeat.cpp` | 64-dim DBoW2 vocab trainer (uses the zero-pad trick). |
| `configs/visual_odometry/vo_euroc_xfeat_lighterglue.yaml` | The headline EuRoC config: XFeat + LighterGlue + KF tuning. |
| `configs/visual_odometry/vo_euroc_xfeat.yaml` | XFeat + MNN fallback config. |
| `configs/map_refinement/mr_euroc_xfeat.yaml` | Map refinement with MNN matcher + 64-dim vocab. |
| `launch/visual_odometry/vo_euroc_xfeat_lighterglue.launch.py` | Headline launch file. |
| `voc/point_voc_L4_xfeat.bin` | Apache-2.0 vocab. 10K words, trained on MH_01 + V1_01 XFeat descriptors. |
| `output/xfeat.onnx`, `output/xfeat.engine` | XFeat (tracked in git: .onnx 2.7 MB; .engine gitignored, regenerable). |
| `output/lighterglue.pt` | LighterGlue TorchScript (tracked: 4.5 MB). Re-export to change N or image_size. |

## Reproducing the bring-up cold

```bash
cd ~/coding/AirSLAM_XFEAT          # branch jazzy-xfeat-ros2

# Engines + LighterGlue .pt (one-shot ~5 min):
bash scripts/get_xfeat_onnx.sh                          # downloads + exports XFeat ONNX
python scripts/patch_onnx_for_trt10.py output/xfeat.onnx
BUILD_ENGINES=xfeat bash scripts/build_engines.sh       # ~30 s

# LighterGlue (needs CUDA torch wheel + cvg/LightGlue checkout):
source ~/.cache/airslam_xfeat/export_venv/bin/activate
# (one-time) cd /tmp && git clone --depth 1 https://github.com/cvg/LightGlue.git cvg_lightglue
python scripts/export_lighterglue_torchscript.py \
  --xfeat-repo ~/.cache/airslam_xfeat/accelerated_features \
  --cvg-lightglue /tmp/cvg_lightglue \
  --output output/lighterglue.pt \
  --num-kpts 512

# Regression guards (must pass — cos ≥ 0.999):
python scripts/numerical_diff_xfeat.py \
  --image ~/datasets/euroc/MH_03_medium/mav0/cam0/data/$(ls ~/datasets/euroc/MH_03_medium/mav0/cam0/data/ | head -1) \
  --onnx output/xfeat_trt10.onnx --engine output/xfeat.engine

# Symlink + build:
ln -sfn ~/coding/AirSLAM_XFEAT ~/ros2_ws/src/air_slam_xfeat
cd ~/ros2_ws && source /opt/ros/jazzy/setup.bash
colcon build --packages-select air_slam_xfeat --symlink-install

# Smoke test (XFeat alone, 564 Hz):
source install/setup.bash
ros2 run air_slam_xfeat test_xfeat \
  --image ~/datasets/euroc/MH_03_medium/mav0/cam0/data/<your.png>
# Expect: 1024 kpts, ~1.77 ms/frame, descriptor norms == 1.000

# Full EuRoC VO + map refinement benchmark (~2 min total):
mkdir -p /tmp/airslam_xfeat_mh03
ros2 launch air_slam_xfeat vo_euroc_xfeat_lighterglue.launch.py \
  dataroot:=~/datasets/euroc/MH_03_medium/mav0 \
  saving_dir:=/tmp/airslam_xfeat_mh03 \
  model_dir:=~/coding/AirSLAM_XFEAT/output \
  visualization:=true   # set false to skip RViz
ros2 launch air_slam_xfeat mr_euroc_xfeat.launch.py \
  map_root:=/tmp/airslam_xfeat_mh03 \
  voc_path:=~/coding/AirSLAM_XFEAT/voc/point_voc_L4_xfeat.bin
evo_ape tum \
  ~/datasets/euroc/MH_03_medium/mav0/state_groundtruth_estimate0/data_tum.txt \
  /tmp/airslam_xfeat_mh03/trajectory_v1.txt -va
# Expect: ATE RMSE ≈ 0.070 m, ~50 FPS, ~400 keyframes
```

## ROS 2 topics (unchanged from jazzy-port)

The `RosPublisher` in `src/ros_publisher.cc` emits:

| Topic | Type | Purpose |
|---|---|---|
| `/AirSLAM/feature` | `sensor_msgs/Image` | Detected keypoints overlay on current frame |
| `/AirSLAM/frame_pose` | `geometry_msgs/PoseStamped` | Current frame pose (high rate) |
| `/AirSLAM/LatestOdometry` | `nav_msgs/Odometry` | Latest odometry (drop-in for Nav2) |
| `/AirSLAM/keyframe` | `sensor_msgs/Image` | Image of the last keyframe |
| `/AirSLAM/odometry` | `nav_msgs/Path` | Trajectory (keyframe path) |
| `/AirSLAM/map` | `visualization_msgs/MarkerArray` | Mappoints (sparse landmarks) |
| `/AirSLAM/mapline` | `visualization_msgs/MarkerArray` | Map lines (XFeat: empty) |
| `/AirSLAM/reloc` | `sensor_msgs/Image` | Relocalization debug image |
| TF `map → diff_bot` (or `world`) | `tf2_msgs/TFMessage` | Pose broadcast |

RViz config at `rviz/vo_jazzy.rviz` is reused for both SuperPoint and
XFeat — same QoS profile, same topic names.

## Realtime status

51.3 FPS end-to-end on EuRoC MH_03 (480x752 stereo + IMU). Camera frame
rates in the wild are typically 20-30 Hz (D455 IR streams) — we're 2x
realtime. The CPU-side g2o local BA is the only thing that can spike a
per-frame to 100-150 ms during keyframe insertion, but the average sits
well under the 33 ms / 30 Hz budget.

## Out of scope (explicit)

- Replacing g2o, ceres, DBoW2 base libraries (would be a multi-week rewrite).
- Re-introducing FP16 / TF32 as defaults (kept on FP32 + `--noTF32`,
  validated cos = 1.000 vs CPU reference).
- Semantic layer / Nvblox fusion / GLIM integration — out of repo.
- Cerebro / Médula / medula_bt — this stack stays standalone.
- LighterGlue ONNX/TRT export — TorchScript covers all needs for now.
  Listed in STATUS.md "Future work" if you ever want to shed the libtorch
  dependency.

---

# Original CLAUDE.md (jazzy-port era, preserved for SuperPoint baseline operations)

## What this is

Fork of [`sair-lab/AirSLAM`](https://github.com/sair-lab/AirSLAM)
(TRO 2025 — point-line VIO with PLNet + LightGlue/SuperGlue +
relocalization) ported to **Ubuntu 24.04 / ROS 2 Jazzy / TensorRT 10.16
/ CUDA 12.6**. The XFeat fork above sits on top of that port.

## Inherited hard rules (still apply)

1. **Algorithm is unchanged**. This is an API-compat port. The SLAM math
   (BA, IMU integration, keyframe management) is the upstream
   `sair-lab/AirSLAM`. Only feature extraction + matching + vocab were
   swapped out for the Apache-2.0 stack.
2. **Never let TRT 8 idioms reappear**. `getNbBindings`,
   `getBindingIndex`, `setBindingDimensions`, `executeV2`,
   `bindingIsInput` are gone. Use the name-keyed TRT 10 API:
   `getNbIOTensors`, `setInputShape`, `setInputTensorAddress`,
   `setOutputTensorAddress`, `enqueueV3(stream)`. The `BufferManager` in
   `3rdparty/tensorrtbuffer/include/buffers.h` is the single source of
   truth.
3. **Two-library split is intentional**. `air_slam_core_lib` must stay
   free of `rclcpp` and `g2o::Line3D`. XFeat lives in core. LighterGlue
   also in core (libtorch has no rclcpp coupling). Anything touching
   `Map` / `RosPublisher` / `g2o_optimization` goes in `air_slam_lib`.
4. **Engines are gitignored** (`*.engine`, `*_trt10.onnx`). Regenerable
   via `scripts/build_engines.sh`.
5. **Default precision is FP32 + `--noTF32` + `--builderOptimizationLevel=5`**.
   This was empirically validated.
6. **`utils.h` is g2o-free**. Anything needing `g2o::Line3D` /
   `Line3DPtr` includes `utils_g2o.h` instead.
7. **The 5-tuple TRT 10 inference pattern** in every `infer()`:
   1. `context_->setInputShape(name, dims)` for every dynamic-shape input.
   2. `BufferManager buffers(engine_, context_.get())`.
   3. `process_input(buffers, ...)` populates host buffers.
   4. `buffers.setTensorAddresses(context_.get())` then
      `buffers.copyInputToDeviceAsync(stream_)`,
      `context_->enqueueV3(stream_)`,
      `buffers.copyOutputToHostAsync(stream_)` (or skip when CUDA
      post-proc reads device buffers directly),
      `cudaStreamSynchronize(stream_)`.
   5. `process_output(buffers, ...)` (or `process_output_cuda()` for XFeat).

   Don't break this order.
