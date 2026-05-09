<h1 align="center">AirSLAM ROS2 (VIO)</h1>

<p align="center">
    <em>ROS 2 Jazzy + TensorRT 10 port — stereo + IMU visual-inertial SLAM, no LiDAR.</em>
</p>

<p align="center">
    <em>Fork of <a href="https://github.com/sair-lab/AirSLAM">sair-lab/AirSLAM</a> (TRO 2025) ported to Ubuntu 24.04 / ROS 2 Jazzy / TensorRT 10.</em>
</p>

<p align="center">
    Original authors:
    <a href = "https://scholar.google.com/citations?user=-p7HvCMAAAAJ&hl=zh-CN">Kuan Xu</a>,
    <a href = "https://github.com/yuefanhao">Yuefan Hao</a>,
    <a href = "https://scholar.google.com/citations?user=XcV_sesAAAAJ&hl=en">Shenghai Yuan</a>,
    <a href = "https://sairlab.org/team/chenw/">Chen Wang</a>,
    <a href = "https://scholar.google.com.sg/citations?user=Fmrv3J8AAAAJ&hl=en">Lihua Xie</a>.
    <a href = "https://arxiv.org/pdf/2408.03520">[paper]</a>
    <a href = "https://xukuanhit.github.io/airslam/">[project site]</a>
</p>

<p align="center">
    Port author: <a href="https://github.com/maikelborys">Maikel</a> (branch <code>jazzy-port</code>).
</p>

---

**AirSLAM** is a hybrid (deep-learning + traditional optimisation) point-line visual SLAM that targets short- and long-term illumination changes. PLNet extracts point + line features in one pass, LightGlue / SuperGlue match them, and a relocalization pipeline lets the robot re-find itself in a previously-built map.

This repository is the **ROS 2 Jazzy / TensorRT 10 port** of upstream `sair-lab/AirSLAM`. The `master` branch is the legacy ROS 1 + ros1_bridge fork for D455 (kept for reference); the current work lives on **`jazzy-port`**.

## Validated results

Running the three-phase pipeline on EuRoC `MH_03_medium` (RTX 4070 8 GB, FP32 + builderOptimizationLevel=5 + `--noTF32`):

| Stage | Metric | Value |
|---|---|---|
| `visual_odometry` | Throughput | **38.3 FPS** stereo+IMU @ 752×480 |
|  | Keyframes / mappoints | 302 / 49,325 |
| `map_refinement` | Loop closures | 127 |
|  | **ATE RMSE post-refinement** | **0.0386 m** (paper: ~0.04 m) |
|  | Mappoint cleanup | 49,325 → 37,980 |
| `relocalization` | Recall on 2,700 queries | **100 %** |
|  | Latency | 49 ms / query (~20 Hz) |

> The numbers in the paper's Table 2 are *post-refinement* (`trajectory_v1.txt`). Raw VO output (`trajectory_v0.txt`) sits at ~0.10 m on this sequence; refinement closes that gap.

## What changed vs upstream

This is an **API-compat port**, not a fork with new features. The algorithm is unchanged. Surface changes:

### Build system
- `package.xml` → format 3, `ament_cmake` (was catkin).
- `CMakeLists.txt` → ament + two-library split:
  - `air_slam_core_lib` — networks + geometry + g2o vertices/edges (no ROS deps).
  - `air_slam_lib` — `Map` / `MapBuilder` / `MapRefiner` / `MapUser` / `RosPublisher` / `g2o_optimization` (ROS-aware).
- `cmake/FindG2O.cmake` filters `NOTFOUND` from `G2O_LIBRARIES` so the slim `apt libg2o-dev` install (no hierarchical/incremental/parser libs) doesn't fail the configure step.
- The original ROS 1 `CMakeLists.txt` is preserved at `CMakeLists.txt.ros1.bak`.

### TensorRT 8 → 10 migration
- `3rdparty/tensorrtbuffer/include/buffers.h` rewritten to the TRT 10 explicit-tensor API: `getNbIOTensors` / `setInputTensorAddress` / `enqueueV3`. `mDeviceBindings` is now a name-keyed `std::map<std::string, void*>`. New helper `setTensorAddresses(context)` wires every I/O device buffer onto the execution context.
- `safe_common.h` got a fix for `roundUp` deduction now that `Dims.d[]` is `int64_t` in TRT 10.
- `sample_entrypoints.h` had `NvCaffeParser.h` and `NvUffParser.h` removed (parsers gone in TRT 10).
- Each network class (`SuperPoint`, `SuperGlue`, `SuperPointLightGlue`, `PLNet`) gained a private `cudaStream_t` and switched from `executeV2(bindings)` → `setInputShape(name, dims)` + `setTensorAddresses(ctx)` + `enqueueV3(stream)` + `cudaStreamSynchronize(stream)`. PLNet's cached binding-index members were removed (TRT 10 keys directly by tensor name).
- A compile-only smoke test at `3rdparty/tensorrtbuffer/test/buffers_compile_test.cpp` runs on every build.
- Two numerical-equivalence harnesses verify network output bit-matches the PyTorch / onnxruntime reference: `scripts/numerical_diff_superpoint.py` (cosine = 1.0) and `scripts/numerical_diff_lightglue.py` (cosine = 1.0 with `--builderOptimizationLevel=5 --noTF32`).

### ROS 1 → ROS 2 Jazzy migration
- `RosPublisher` rewritten on top of `rclcpp::Publisher<T>::SharedPtr` and `tf2_ros::TransformBroadcaster`. All ten upstream publishers (`/AirSLAM/feature`, `/AirSLAM/frame_pose`, `/AirSLAM/keyframe`, `/AirSLAM/odometry`, `/AirSLAM/map`, `/AirSLAM/mapline`, `/AirSLAM/reloc/{trajectory,pose,matches}`, `/AirSLAM/LatestOdometry`) preserved 1:1.
- `MapBuilder`, `MapRefiner`, `MapUser` constructors now take `rclcpp::Node::SharedPtr` instead of `ros::NodeHandle`.
- `MapRefiner::PubMap` and `MapUser::Relocalization` had their `ros::Time::now()` and `ros::Rate` replaced with `std::chrono` and `rclcpp::Rate`.
- The four executables (`visual_odometry`, `map_refinement`, `relocalization`, `test_feature`) rewritten with `rclcpp::init` / `Node::declare_parameter` / `Node::get_parameter` / `rclcpp::ok` / `rclcpp::shutdown`.
- Every ROS 1 `.launch` XML converted to a ROS 2 `.launch.py` (10 files total).
- A new `rviz/vo_jazzy.rviz` covers all topics with explicit QoS (`Reliability=Reliable`, `Durability=Volatile`, `History=Keep Last`, depth=10) so RViz2 subscribes cleanly.

### Engine pipeline
- `scripts/patch_onnx_for_trt10.py` inserts `Cast Int32 → Int64` nodes on `Concat` / `Mul` / `Add` / `Where` / etc. ops that TRT 10's stricter ONNX importer rejects (zero patches needed for LightGlue and PLNet, 1 for SuperPoint, 156 for SuperGlue).
- `scripts/build_engines.sh` regenerates the five `.engine` files from the patched ONNX with `trtexec`. Defaults to `--noTF32 --builderOptimizationLevel=5` for the most accurate kernels (overridable via `PRECISION_FLAG=` and `OPT_LEVEL=` env vars).

### Refactor: `utils.h` decoupling
- The `g2o::Line3D`-dependent helpers moved to a new `include/utils_g2o.h`. The TRT 10 network sources (super_point, plnet, etc.) now compile without `g2o` on the include path, which keeps `air_slam_core_lib` ROS-and-g2o-free.

## Install (Ubuntu 24.04 + ROS 2 Jazzy)

System dependencies — one-time:

```bash
sudo apt install -y \
  ros-jazzy-desktop ros-jazzy-cv-bridge ros-jazzy-image-transport \
  ros-jazzy-tf2 ros-jazzy-tf2-ros ros-jazzy-tf2-geometry-msgs \
  libg2o-dev libgoogle-glog-dev libgflags-dev \
  libopencv-dev libeigen3-dev libyaml-cpp-dev libboost-serialization-dev
```

Plus a working **CUDA 12.x + TensorRT 10** install (host has CUDA 12.6 + TRT 10.16). For ONNX patching:

```bash
uv venv ~/.airslam_venv --python 3.12
source ~/.airslam_venv/bin/activate
uv pip install onnx onnx-graphsurgeon onnxruntime opencv-python numpy
```

Clone + symlink into a colcon workspace:

```bash
git clone -b jazzy-port https://github.com/maikelborys/AirSLAM_ROS_D455.git ~/coding/AirSLAM
ln -s ~/coding/AirSLAM ~/ros2_ws/src/air_slam
```

Build:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select air_slam --symlink-install
```

Generate engines (≈5 min on RTX 4070):

```bash
source ~/.airslam_venv/bin/activate
cd ~/coding/AirSLAM
python scripts/patch_onnx_for_trt10.py output/superpoint_v1_sim_int32.onnx \
                                       output/superpoint_lightglue.onnx \
                                       output/superglue_outdoor_sim_int32.onnx \
                                       output/superglue_indoor_sim_int32.onnx \
                                       output/plnet_s0.onnx output/plnet_s1.onnx
bash scripts/build_engines.sh
```

## Run

EuRoC `MH_03_medium` end-to-end pipeline (the validation flow):

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

# 1. Visual-inertial odometry — builds an initial map.
ros2 launch air_slam vo_euroc.launch.py \
  dataroot:=/path/to/MH_03_medium/mav0 \
  saving_dir:=/tmp/airslam_mh03 \
  model_dir:=/home/maikel/coding/AirSLAM/output

# 2. Offline map refinement (loop closure + global BA + map merge).
ros2 launch air_slam mr_euroc.launch.py \
  map_root:=/tmp/airslam_mh03 \
  model_dir:=/home/maikel/coding/AirSLAM/output

# 3. Relocalization — query images against the refined map.
ros2 launch air_slam reloc_euroc.launch.py \
  map_root:=/tmp/airslam_mh03 \
  dataroot:=/path/to/MH_03_medium/mav0/cam0/data \
  model_dir:=/home/maikel/coding/AirSLAM/output

# 4. Compute ATE.
evo_ape tum  /path/to/MH_03_medium/mav0/state_groundtruth_estimate0/data_tum.txt \
             /tmp/airslam_mh03/trajectory_v1.txt -va
```

All launches accept `visualization:=false` to suppress RViz2.

## RViz topics (preconfigured in `rviz/vo_jazzy.rviz`)

| Topic | Type | Display |
|---|---|---|
| `/AirSLAM/feature` | `sensor_msgs/Image` | Live image with feature/match overlay |
| `/AirSLAM/frame_pose` | `geometry_msgs/PoseStamped` | Current camera pose (axes) |
| `/AirSLAM/odometry` | `nav_msgs/Path` | Trajectory polyline (green) |
| `/AirSLAM/keyframe` | `geometry_msgs/PoseArray` | Keyframe axes (red) |
| `/AirSLAM/map` | `sensor_msgs/PointCloud` | Mappoints (yellow points) |
| `/AirSLAM/mapline` | `visualization_msgs/Marker` | 3D line segments |
| `/AirSLAM/LatestOdometry` | `nav_msgs/Odometry` | Per-frame odometry |
| `/AirSLAM/reloc/trajectory` | `Marker` | Reloc query trail (green spheres) |
| `/AirSLAM/reloc/pose` | `PoseStamped` | Reloc current pose (cyan axes) |
| `/AirSLAM/reloc/matches` | `Marker` | Camera↔mappoint match lines |

TF tree: `map → camera`, broadcast by `RosPublisher` once per frame.

## Original supported sequences

The `launch/` folder still ships the same configuration variants as upstream — EuRoC dark, OIVIO, TartanAir, UMA Bumblebee — only with `.launch.py` versions. Adapt `dataroot:=` to your local paths.

## Citation

```bibtex
@article{xu2024airslam,
  title = {{AirSLAM}: An Efficient and Illumination-Robust Point-Line Visual SLAM System},
  author = {Xu, Kuan and Hao, Yuefan and Yuan, Shenghai and Wang, Chen and Xie, Lihua},
  journal = {IEEE Transactions on Robotics (TRO)},
  year = {2024},
  url = {https://arxiv.org/abs/2408.03520},
  code = {https://github.com/sair-lab/AirSLAM},
}
```

## License

Original AirSLAM license (see `LICENSE.md`). Port additions inherit the same.
