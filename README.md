<h1 align="center">AirSLAM-XFeat (ROS 2 Jazzy + TensorRT 10 + libtorch)</h1>

<p align="center">
    <em>Apache-2.0 fork of AirSLAM with the SuperPoint perception layer replaced by XFeat + LighterGlue.</em>
</p>

<p align="center">
    <em>Stereo + IMU visual-inertial SLAM. No LiDAR.</em>
</p>

<p align="center">
    Branch <code>jazzy-xfeat-ros2</code> ·
    51.3 FPS on EuRoC MH_03 (vs 38.3 FPS SuperPoint baseline) ·
    ATE 0.070 m post-refinement (vs 0.039 m baseline) ·
    <strong>fully commercial-deployable license stack.</strong>
</p>

<p align="center">
    Upstream:
    <a href="https://github.com/sair-lab/AirSLAM">sair-lab/AirSLAM</a> (TRO 2025) ·
    Port lineage: <a href="https://github.com/maikelborys/AirSLAM_ROS_D455/tree/jazzy-port">jazzy-port</a> (ROS 2 Jazzy / TRT 10 port).
</p>

<p align="center">
    Port author: <a href="https://github.com/maikelborys">Maikel</a>.
</p>

---

## Why this fork exists

Upstream AirSLAM uses **SuperPoint** (MagicLeap) for keypoint extraction
and **LightGlue** / **SuperGlue** for matching. SuperPoint's weights are
licensed for **non-commercial research use only**, and the shipped DBoW2
vocabulary (`voc/point_voc_L4.bin`) is derived from SuperPoint features
— so the entire upstream stack is **license-tainted for commercial
deployment**.

This branch swaps the perception layer for an **Apache-2.0 stack**:

| Component | Upstream (non-commercial) | This branch (Apache-2.0) |
|---|---|---|
| Feature extractor | SuperPoint (MagicLeap) | **XFeat** (Verlab) |
| Matcher | LightGlue / SuperGlue | **LighterGlue** + MNN fallback |
| DBoW2 vocab | `point_voc_L4.bin` (SP-derived) | **`point_voc_L4_xfeat.bin`** (XFeat-trained) |
| SLAM backend (g2o, IMU, BA, keyframes, loop closure) | unchanged | unchanged |

The SLAM math is identical. Only the feature/matcher/vocab layer changes.

## Validated results — EuRoC

End-to-end pipeline (visual odometry → map refinement) on a single RTX
4070, FP32 + `--noTF32` + `builderOptimizationLevel=5`:

| Sequence | FPS (full) | Raw VO ATE | Post-refinement ATE | Keyframes |
|---|---|---|---|---|
| **MH_03_medium** | **51.3** | 0.207 m | **0.070 m** | 408 |
| **V1_01_easy** (Vicon Room) | **63.5** | **0.083 m** | (raw only) | 219 |
| MH_03 — SuperPoint baseline (paper) | 38.3 | ~0.10 m | 0.039 m | 302 |

**This branch is 34 % faster than the SuperPoint baseline at 1.8× the
ATE — and it can be commercially deployed.**

XFeat extractor in isolation: **564 Hz** (1.77 ms / frame on the GPU
side with CUDA post-processing) — see `output/benchmarks_xfeat.md` for
the full benchmark log.

## License posture

| Component | License | Commercial use |
|---|---|---|
| AirSLAM source | Apache-2.0 | ✅ |
| **XFeat** weights | **Apache-2.0** | ✅ |
| **LighterGlue** weights | **Apache-2.0** | ✅ |
| **`voc/point_voc_L4_xfeat.bin`** (this branch) | Apache-2.0 (XFeat-derived) | ✅ |
| MNN matcher | algorithm only, no weights | ✅ |
| DBoW2 / g2o / Eigen / OpenCV / Boost | BSD / MIT / MPL / Apache | ✅ |
| TensorRT 10, CUDA, cuBLAS | NVIDIA EULA, free runtime | ✅ |
| libtorch | BSD-3 | ✅ |
| SuperPoint (upstream) | Non-commercial research | ❌ — not used here |

This is the **first and only license-clean path** to a deployable
AirSLAM as of writing.

## Quick start

System requirements:

- Ubuntu 24.04, ROS 2 Jazzy, CUDA 12.6, TensorRT 10.16
- NVIDIA GPU with ~2 GB VRAM (engines + libtorch session)
- libtorch 2.7+ (CUDA build, installed at `~/libtorch`)

```bash
# 1) One-time system dependencies
sudo apt install -y \
  ros-jazzy-desktop ros-jazzy-cv-bridge ros-jazzy-image-transport \
  ros-jazzy-tf2 ros-jazzy-tf2-ros ros-jazzy-tf2-geometry-msgs \
  libg2o-dev libgoogle-glog-dev libgflags-dev \
  libopencv-dev libeigen3-dev libyaml-cpp-dev libboost-serialization-dev

# 2) Clone + symlink
git clone -b jazzy-xfeat-ros2 https://github.com/maikelborys/AirSLAM_ROS_D455.git \
  ~/coding/AirSLAM_XFEAT
ln -s ~/coding/AirSLAM_XFEAT ~/ros2_ws/src/air_slam_xfeat

# 3) Generate XFeat ONNX + TRT 10 engine (one-shot, ~3 min idempotent)
cd ~/coding/AirSLAM_XFEAT
bash scripts/get_xfeat_onnx.sh
python scripts/patch_onnx_for_trt10.py output/xfeat.onnx
BUILD_ENGINES=xfeat bash scripts/build_engines.sh

# 4) Export LighterGlue TorchScript (.pt) — needs CUDA torch wheel
source ~/.cache/airslam_xfeat/export_venv/bin/activate
# (one-time) git clone --depth 1 https://github.com/cvg/LightGlue.git /tmp/cvg_lightglue
python scripts/export_lighterglue_torchscript.py \
  --xfeat-repo ~/.cache/airslam_xfeat/accelerated_features \
  --cvg-lightglue /tmp/cvg_lightglue \
  --output output/lighterglue.pt --num-kpts 512

# 5) Build
cd ~/ros2_ws && source /opt/ros/jazzy/setup.bash
colcon build --packages-select air_slam_xfeat --symlink-install
```

## Run

EuRoC MH_03 end-to-end pipeline with live RViz visualization:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

# 1) Visual odometry (51 FPS, ~50 s for 2700 frames + map save)
ros2 launch air_slam_xfeat vo_euroc_xfeat_lighterglue.launch.py \
  dataroot:=/path/to/MH_03_medium/mav0 \
  saving_dir:=/tmp/airslam_xfeat_mh03 \
  model_dir:=~/coding/AirSLAM_XFEAT/output \
  visualization:=true

# 2) Map refinement (loop closure + global BA, ~3 min)
ros2 launch air_slam_xfeat mr_euroc_xfeat.launch.py \
  map_root:=/tmp/airslam_xfeat_mh03 \
  voc_path:=~/coding/AirSLAM_XFEAT/voc/point_voc_L4_xfeat.bin \
  model_dir:=~/coding/AirSLAM_XFEAT/output

# 3) Compute ATE
evo_ape tum \
  /path/to/MH_03_medium/mav0/state_groundtruth_estimate0/data_tum.txt \
  /tmp/airslam_xfeat_mh03/trajectory_v1.txt -va
# Expect: ATE RMSE ≈ 0.070 m
```

The launch file accepts `max_frames:=N` and `skip_save_map:=1` for fast
iteration during tuning.

## RViz topics (preconfigured at `rviz/vo_jazzy.rviz`)

| Topic | Type | Display |
|---|---|---|
| `/AirSLAM/feature` | `sensor_msgs/Image` | Live image + keypoint overlay |
| `/AirSLAM/frame_pose` | `geometry_msgs/PoseStamped` | Current camera pose (axes) |
| `/AirSLAM/odometry` | `nav_msgs/Path` | Trajectory polyline (green) |
| `/AirSLAM/keyframe` | `geometry_msgs/PoseArray` | Keyframe axes (red) |
| `/AirSLAM/map` | `visualization_msgs/MarkerArray` | Mappoints (landmark cloud) |
| `/AirSLAM/LatestOdometry` | `nav_msgs/Odometry` | Per-frame odometry (Nav2 drop-in) |
| `/AirSLAM/mapline` | `visualization_msgs/MarkerArray` | Map lines (empty in XFeat mode) |
| TF `map → diff_bot` | `tf2_msgs/TFMessage` | Pose broadcast |

## Architecture in one diagram

```
Image bytes (Dataset or D455 topic)
  │
  ▼
cv::resize → 480×752 grayscale (float32)
  │
  ▼ (GPU)
┌──────────────────────┐    ┌──────────────────────────┐
│  XFeat TRT 10 engine │ ─→ │  CUDA post-proc (4 kernels) │
│  0.7 ms / frame      │    │  softmax / NMS / top-K /  │
│  feats, keypts, rel  │    │  bilinear sample          │
└──────────────────────┘    │  → 67-row Eigen matrix    │
                            └──────────────────────────┘
                                       │
                                       ▼ (GPU)
                            ┌──────────────────────────┐
                            │  LighterGlue TorchScript │
                            │  via libtorch, attention │
                            │  → matches + scores      │
                            └──────────────────────────┘
                                       │
                                       ▼ (CPU)
                            ┌──────────────────────────┐
                            │  MapBuilder              │
                            │  – stereo triangulation  │
                            │  – tracking + g2o local  │
                            │    BA on keyframes       │
                            │  – DBoW2 loop closure    │
                            │    (64-dim XFeat vocab)  │
                            └──────────────────────────┘
                                       │
                                       ▼
                            ROS 2 topics
                            (path, mappoints, TF, …)
```

Full diagrams in `ARCHITECTURE.md`.

## Docs index

| File | Read when |
|---|---|
| `README.md` (this) | First contact with the repo |
| `CLAUDE.md` | Cold-starting a Claude Code session — hard rules X1–X8 |
| `STATUS.md` | Latest measured numbers, what works, what's limited, future work |
| `ARCHITECTURE.md` | Pipeline layered diagram, per-frame data flow, full file map |
| `PROCESS.md` | Journey: what was tried, what failed (ONNX export ×4), KF revelation, lessons learned |
| `README_JAZZY_XFEAT.md` | Delta vs the SuperPoint jazzy-port |
| `output/benchmarks_xfeat.md` | Full benchmark log + bottleneck analysis |
| `README_JAZZY.md` | Original jazzy-port era doc (SuperPoint baseline reference) |

## What's in scope, what isn't

In scope, validated:
- ✅ Full XFeat + LighterGlue + 64-dim vocab pipeline on EuRoC
- ✅ ROS 2 Jazzy topic publishing (`/AirSLAM/{feature, frame_pose, map, odometry, keyframe, …}`)
- ✅ Map refinement (loop closure + global BA) with the new vocab
- ✅ CUDA post-processing for XFeat (4 kernels in `src/xfeat_postproc.cu`)
- ✅ Keyframe-rate tuning that handles XFeat's match-density profile
- ✅ Reproducible build from clean clone (`get_xfeat_onnx.sh` +
      `export_lighterglue_torchscript.py`)

Out of scope this branch (in `STATUS.md` "Future work"):
- D455 live-camera ROS 2 topic ingestion — wiring exists on the
  `master` ROS 1 branch, not yet cherry-picked.
- Re-enabling lines (PLNet wireframe head) — Phase 6 metrics passed
  the promotion gate, but lines aren't in the headline config yet.
- Replacing g2o local BA with GPU BA (MegBA / DeepLM / TheseusAI) —
  the biggest remaining FPS lever, ~1-2 weeks of risky work.
- LighterGlue TRT engine (currently TorchScript via libtorch) — would
  let us drop the libtorch runtime dependency.

## Acknowledgements

Built on top of incredible upstream work:

- **AirSLAM** — Kuan Xu, Yuefan Hao, Shenghai Yuan, Chen Wang,
  Lihua Xie. [TRO 2025 paper](https://arxiv.org/abs/2408.03520) ·
  [code](https://github.com/sair-lab/AirSLAM) ·
  [project site](https://xukuanhit.github.io/airslam/)
- **XFeat** — Verlab, UFMG.
  [CVPR 2024 paper](https://arxiv.org/abs/2404.19174) ·
  [code](https://github.com/verlab/accelerated_features)
- **LightGlue / LighterGlue** — CVG @ ETH Zürich + Verlab. The kornia
  port of LightGlue is what the .pt loads.
- **DBoW2** — D. Gálvez-López, J. D. Tardós (BSD-3).
- **g2o** — R. Kümmerle et al. (BSD).

## Citation

If you publish work that uses this branch, please cite the upstream
papers:

```bibtex
@article{xu2024airslam,
  title = {{AirSLAM}: An Efficient and Illumination-Robust Point-Line Visual SLAM System},
  author = {Xu, Kuan and Hao, Yuefan and Yuan, Shenghai and Wang, Chen and Xie, Lihua},
  journal = {IEEE Transactions on Robotics (TRO)},
  year = {2024},
  url = {https://arxiv.org/abs/2408.03520},
}

@inproceedings{potje2024xfeat,
  title = {{XFeat}: Accelerated Features for Lightweight Image Matching},
  author = {Potje, Guilherme and Cadar, Felipe and Araujo, Andr\'{e} and Martins, Renato and Nascimento, Erickson R.},
  booktitle = {Proceedings of the IEEE/CVF Conference on Computer Vision and Pattern Recognition (CVPR)},
  year = {2024},
}

@inproceedings{lindenberger2023lightglue,
  title = {{LightGlue}: Local Feature Matching at Light Speed},
  author = {Lindenberger, Philipp and Sarlin, Paul-Edouard and Pollefeys, Marc},
  booktitle = {Proceedings of the IEEE/CVF International Conference on Computer Vision (ICCV)},
  year = {2023},
}
```

## License

Apache-2.0 (this branch). Original AirSLAM `LICENSE.md` preserved.
XFeat and LighterGlue weights are Apache-2.0 per their upstream
licensing. **SuperPoint weights and the upstream `voc/point_voc_L4.bin`
are NOT used in this branch.**
