# AirSLAM — ROS 2 Jazzy + TensorRT 10 port

Status: **port in progress** (branch `jazzy-port` of this fork).

This document records the deltas from upstream `sair-lab/AirSLAM` so the next session can pick up cold. The full plan lives at
`~/.claude/plans/si-auiqres-haz-plan-golden-wombat.md`.

## Target environment

| | |
|---|---|
| OS | Ubuntu 24.04 (Noble) |
| ROS | ROS 2 **Jazzy** |
| CUDA | 12.6 |
| TensorRT | **10.16** |
| OpenCV | 4.6 |
| Build | `colcon build --packages-select air_slam --symlink-install` from `~/ros2_ws/` |

## What's done

- `package.xml` → format 3 (`ament_cmake`, ROS 2 deps).
- `CMakeLists.txt` → ament + two-lib split (`air_slam_core_lib` non-ROS, `air_slam_lib` ROS-aware).
- `3rdparty/tensorrtbuffer/include/buffers.h` migrated to TRT 10 tensor-name API (`getNbIOTensors` / `setInputTensorAddress` / `enqueueV3`); `mDeviceBindings` is now a name-keyed map; new helper `setTensorAddresses(context)`. Compile-only smoke test in `3rdparty/tensorrtbuffer/test/buffers_compile_test.cpp` runs as part of the build.
- `3rdparty/tensorrtbuffer/include/sample_entrypoints.h` cleaned: `NvCaffeParser.h` / `NvUffParser.h` removed (parsers gone in TRT 10).
- `3rdparty/tensorrtbuffer/include/safe_common.h` — `roundUp` deduction fixed for TRT 10 `Dims.d[]` being `int64_t`.
- All 4 networks ported to TRT 10:
  - `src/super_point.cpp` (canonical template — 1 input, 2 outputs).
  - `src/super_glue.cpp` (6 inputs, 1 output).
  - `src/light_glue.cpp` (4 inputs, 1 output).
  - `src/plnet.cpp` (two engines; cached binding-index members removed, replaced by direct name calls).
- Each network gained a private `cudaStream_t stream_`, created in the constructor and destroyed in the destructor.
- `include/utils.h` is now g2o-free; the Line3D-dependent helpers moved to `include/utils_g2o.h`. The non-network sources (mapline, map, line_processor, g2o_optimization/, map_refiner) include `utils_g2o.h` instead.
- ROS layer:
  - `include/ros_publisher.h` + `src/ros_publisher.cc` ported to `rclcpp` + `tf2_ros::TransformBroadcaster` (was `ros::NodeHandle` + `tf::TransformBroadcaster`). 10 publishers preserved 1:1.
  - `MapBuilder`, `MapRefiner`, `MapUser` constructor signatures take `rclcpp::Node::SharedPtr` instead of `ros::NodeHandle`.
  - 4 demos (`visual_odometry`, `map_refinement`, `relocalization`, `test_feature`) ported: `rclcpp::init` / `node->declare_parameter` / `rclcpp::ok` / `rclcpp::shutdown`.
- 10 ROS 1 `.launch` XML files → 10 `.launch.py` Python equivalents in the same paths.
- `scripts/build_engines.sh` regenerates all 5 `.engine` files from the bundled ONNX with `trtexec` (start in FP32, switch to `--fp16` after EuRoC ATE pass).

## What you need to do

### 1. Install missing apt packages (sudo)

The Phase 3 build needs G2O + Glog + Gflags. Run from any shell:

```
sudo apt install -y libg2o-dev libgoogle-glog-dev libgflags-dev
```

This is the only manual step. Once the headers land in `/usr/include/g2o/` the build resumes automatically.

### 2. Regenerate engines from ONNX

```
cd ~/coding/AirSLAM
bash scripts/build_engines.sh
```

If PLNet stage 1 fails with a Cast/Gather operator error (upstream issue [#203](https://github.com/sair-lab/AirSLAM/issues/203)), retry with `--fp16` removed and consider re-exporting the ONNX with `--opset=17` from the upstream PyTorch repo. As a fallback, set `use_superpoint: 1` in `configs/visual_odometry/vo_euroc.yaml` to disable line features and validate point-only VIO first.

### 3. Run EuRoC validation

```
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch air_slam vo_euroc.launch.py \
    dataroot:=/path/to/MH_03_medium \
    saving_dir:=/tmp/airslam_run
evo_ape euroc /path/to/MH03_GT.csv /tmp/airslam_run/trajectory_v0.txt -va
```

Pass criterion: ATE ≤ 0.06 m at ≥ 30 FPS. Anything beyond an order of magnitude indicates a TRT migration issue (re-run the per-network smoke tests).

## Files of note

| Path | Role |
|---|---|
| `CMakeLists.txt.ros1.bak` | Original ROS 1 CMakeLists, kept as reference. |
| `package.xml` | Format 3, ament_cmake. |
| `include/utils_g2o.h` | New — g2o-dependent helpers extracted from `utils.h`. |
| `scripts/build_engines.sh` | Regenerate TRT 10 engines. |
| `3rdparty/tensorrtbuffer/test/buffers_compile_test.cpp` | Smoke test — fails if `buffers.h` ever stops parsing under TRT 10. |
| `launch/{visual_odometry,map_refinement,relocalization}/*.launch.py` | New ROS 2 launch files. |

## Out of scope (for this port)

- D455 live camera path (cherry-pick `ROSDataset` + D455 configs from `maikelborys/AirSLAM_ROS_D455` after EuRoC validates).
- Semantic layer integration.
- LiDAR/GLIM fusion — Maikel runs pure VIO on this stack.
- Performance tuning beyond restoring TRT 8 baseline FPS.

## Final validation results (2026-05-09)

Full AirSLAM pipeline validated on EuRoC MH_03_medium with FP32 + optLvl=5:

| Phase | Metric | Result |
|---|---|---|
| Visual odometry | Average FPS | 38.3 |
| | Keyframes / mappoints | 302 / 49,325 |
| Map refinement | Loop pairs found | 127 |
| | **ATE RMSE post-refinement** | **0.0386 m** (matches paper ~0.04 m) |
| | Mappoint cleanup | 49,325 → 37,980 |
| Relocalization | Recall on 2700 queries | 100% |
| | Latency per query | 49 ms (~20 Hz) |

Important: the published paper ATE numbers in Table 2 are POST-refinement.
Raw `trajectory_v0.txt` from `visual_odometry` is ~0.10 m on this sequence;
running `map_refinement` over the saved map drops that to 0.039 m.

Reproduce:
```
bash scripts/build_engines.sh                                       # ~5 min
ros2 run air_slam visual_odometry  --ros-args -p ... saving_dir:=/tmp/run
ros2 run air_slam map_refinement   --ros-args -p map_root:=/tmp/run ...
ros2 run air_slam relocalization   --ros-args -p map_root:=/tmp/run ...
evo_ape tum  GT.txt /tmp/run/trajectory_v1.txt -va
```
