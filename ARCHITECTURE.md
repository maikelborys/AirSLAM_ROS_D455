# ARCHITECTURE — AirSLAM-XFeat

Pipeline-level map of the system. For module-level conventions and hard
rules see `CLAUDE.md`; for measured numbers see `STATUS.md` and
`output/benchmarks_xfeat.md`.

## Layered view

```
┌───────────────────────────────────────────────────────────────────────┐
│                            VO / map_refinement / relocalization       │
│                                (demo/*.cpp executables)               │
└──────────────────────────────────┬────────────────────────────────────┘
                                   │
┌──────────────────────────────────┴────────────────────────────────────┐
│   air_slam_lib (ROS 2-aware: rclcpp, RosPublisher, MapBuilder,        │
│   MapRefiner, MapUser, g2o_optimization)                              │
│                                                                       │
│   ┌─────────────────┐  ┌──────────────────┐  ┌────────────────────┐  │
│   │  MapBuilder     │  │  MapRefiner      │  │  MapUser           │  │
│   │  (per-frame VO) │  │  (loop closure + │  │  (relocalization)  │  │
│   │                 │  │   global BA)     │  │                    │  │
│   └────────┬────────┘  └────────┬─────────┘  └────────┬───────────┘  │
└────────────┼────────────────────┼──────────────────────┼─────────────┘
             │                    │                      │
             ▼                    ▼                      ▼
┌──────────────────────────────────────────────────────────────────────┐
│   air_slam_core_lib (NO rclcpp, NO g2o::Line3D — pure SLAM math)     │
│                                                                      │
│   FeatureDetector  ── dispatch on feature_extractor ──┐              │
│        │                                              │              │
│        ├── XFeat (mode 2, default)                    │              │
│        │     ├── TRT 10 engine forward (GPU)          │              │
│        │     └── CUDA post-proc (4 kernels)           │              │
│        │                                              │              │
│        ├── SuperPoint (mode 1, A/B fallback)          │              │
│        │     └── TRT 10 engine forward (GPU)          │              │
│        │                                              │              │
│        └── PLNet (mode 0, with lines — unused now)    │              │
│                                                       │              │
│   PointMatcher ── dispatch on matcher ────────────────┤              │
│        │                                              │              │
│        ├── LighterGlue (mode 3, default for XFeat)    │              │
│        │     └── TorchScript .pt via libtorch (GPU)   │              │
│        │                                              │              │
│        ├── MNN+Lowe (mode 2)                          │              │
│        │     └── cuBLAS SGEMM cosine matrix (GPU)     │              │
│        │                                              │              │
│        ├── LightGlue (mode 0, SuperPoint A/B)         │              │
│        │     └── TRT 10 engine (GPU)                  │              │
│        │                                              │              │
│        └── SuperGlue (mode 1, SuperPoint A/B)         │              │
│              └── TRT 10 engine (GPU)                  │              │
│                                                       │              │
│   Database (DBoW2 wrapper)                            │              │
│        └── SuperpointVocabulary (L=256, but XFeat ────┘              │
│            descriptors are zero-padded to 256 dims)                  │
└──────────────────────────────────────────────────────────────────────┘
             │                    │                      │
             ▼                    ▼                      ▼
┌──────────────────────────────────────────────────────────────────────┐
│   3rdparty (vendored)                                                │
│                                                                      │
│   tensorrtbuffer (TRT 10 name-keyed BufferManager)                   │
│   DBoW2 (templated bag-of-words)                                     │
└──────────────────────────────────────────────────────────────────────┘
             │                    │                      │
             ▼                    ▼                      ▼
┌──────────────────────────────────────────────────────────────────────┐
│   System libs                                                        │
│                                                                      │
│   TensorRT 10.16   CUDA 12.6   libtorch 2.7   cuBLAS                 │
│   OpenCV 4.6       Eigen       Boost          g2o   yaml-cpp         │
│   rclcpp           tf2_ros     message_filters     cv_bridge          │
└──────────────────────────────────────────────────────────────────────┘
```

## Per-frame data flow (XFeat mode)

```
       Image bytes from Dataset / D455 topic
                       │
                       ▼
         ┌──────────────────────────────┐
         │ cv::resize -> 480x752 grayscale│
         │ normalize / 255.0              │
         └──────────────────────────────┘
                       │
              ┌────────┴───────┐
              ▼                ▼
         Left image       Right image
              │                │
              ▼                ▼
    ┌────────────────┐ ┌────────────────┐
    │  XFeat::infer  │ │  XFeat::infer  │
    │  ───────────── │ │  (called only  │
    │  1. setInputShape   on KF init or │
    │  2. BufferManager   non-init KF)  │
    │  3. process_input  │ │              │
    │  4. enqueueV3 ─── GPU TRT 10 ─── feats, keypts, rel (device)
    │  5. process_output_cuda          │
    │     - softmax+unfold (CUDA)      │
    │     - NMS dilate     (CUDA)      │
    │     - emit candidates(CUDA)      │
    │     - top-K          (host)      │
    │     - sample desc    (CUDA)      │
    │     - L2-norm        (CUDA)      │
    │  → 67xN Eigen matrix (host)      │
    └────────┬───────┘ └────────┬───────┘
             │                  │
             ▼                  ▼
       259xN padded       259xN padded
       (rows 0-2: xy+score, rows 3-66: XFeat,
        rows 67-258: zeros for vocab compat)
             │                  │
             └────────┬─────────┘
                      ▼
            ┌──────────────────────┐
            │  PointMatcher        │
            │  ──────────────────  │
            │  if matcher == 3:    │
            │    LighterGlue       │
            │    .pt via libtorch  │
            │  if matcher == 2:    │
            │    cuBLAS GEMM +     │
            │    Lowe + mutual NN  │
            │  → cv::DMatch[]      │
            └──────────────────────┘
                      │
                      ▼ (stereo matches)
            ┌──────────────────────┐
            │  Frame.AddRightFeatures
            │  triangulate mappoints (CPU)
            └──────────┬───────────┘
                       │
                       ▼
            ┌──────────────────────┐
            │  MapBuilder::AddInput│
            │  ──────────────────  │
            │  - Track (PointMatcher
            │     against last KF)  │
            │  - AddKeyframeCheck:  │
            │     return 0/1/2      │
            │  - if KF: g2o local BA
            │     (CPU, single-thread)
            └──────────┬───────────┘
                       │
                       ▼
            ┌──────────────────────┐
            │  RosPublisher        │
            │  ──────────────────  │
            │  /AirSLAM/feature    │
            │  /AirSLAM/frame_pose │
            │  /AirSLAM/odometry   │
            │  /AirSLAM/map        │
            │  /AirSLAM/keyframe   │
            │  TF map → robot      │
            └──────────────────────┘
```

## File map (XFeat-specific additions)

```
~/coding/AirSLAM_XFEAT/
│
├── include/
│   ├── xfeat.h                  XFeat wrapper header
│   ├── xfeat_postproc.h         CUDA post-proc opaque-handle API
│   ├── lighter_glue.h           LighterGlue pimpl (no libtorch leak)
│   ├── feature_detector.h       Dispatch on feature_extractor
│   ├── point_matcher.h          Dispatch on PointMatcherKind
│   ├── read_configs.h           XFeatConfig + extractor/matcher enums
│   └── bow/                     DBoW2 wrapper (FSuperpoint reused
│                                with zero-padded XFeat descriptors)
│
├── src/
│   ├── xfeat.cpp                TRT 10 5-step infer + CUDA postproc dispatch
│   ├── xfeat_postproc.cu        4 CUDA kernels (softmax/NMS/emit/sample)
│   ├── lighter_glue.cc          TorchScript loader + forward
│   ├── feature_detector.cc      DetectXFeat: 67→259 zero-pad
│   ├── point_matcher.cc         MNN/LighterGlue/LightGlue dispatch
│   ├── ros_publisher.cc         rclcpp ROS 2 topic emitter
│   ├── map_builder.cc           VO main loop (CPU + GPU calls)
│   ├── map_refiner.cc           Loop closure + global BA (CPU)
│   ├── map_user.cc              Relocalization (CPU + GPU calls)
│   └── bow/                     DBoW2 wrapping
│
├── demo/
│   ├── visual_odometry.cpp      VO executable (--max-frames + --skip_save_map)
│   ├── map_refinement.cpp       Loop closure executable
│   ├── relocalization.cpp       Reloc executable
│   ├── test_xfeat.cpp           Standalone XFeat smoke test (564 Hz)
│   └── train_voc_xfeat.cpp      64-dim DBoW2 vocab trainer
│
├── scripts/
│   ├── get_xfeat_onnx.sh        Idempotent orchestrator (clones upstream,
│   │                            builds throwaway uv venv, exports)
│   ├── export_xfeat_onnx.py     PyTorch → ONNX (static shape)
│   ├── export_lighterglue_torchscript.py   PyTorch → .pt (Path D)
│   ├── export_lighterglue_onnx.py          (BLOCKED — kept for reference)
│   ├── patch_onnx_for_trt10.py  Int32→Int64 cast inserts
│   ├── build_engines.sh         trtexec wrapper (BUILD_ENGINES allowlist)
│   ├── numerical_diff_xfeat.py  cos ≥ 0.999 regression guard
│   └── numerical_diff_superpoint.py  (legacy)
│
├── configs/visual_odometry/
│   ├── vo_euroc_xfeat_lighterglue.yaml   ⭐ HEADLINE config (KF-tuned)
│   ├── vo_euroc_xfeat.yaml               XFeat + MNN fallback
│   ├── vo_euroc.yaml                     SuperPoint baseline
│   └── vo_*_dark.yaml, vo_oivio.yaml, vo_tartanair.yaml, ...
│
├── configs/map_refinement/
│   ├── mr_euroc_xfeat.yaml      64-dim vocab, MNN matcher for loop matching
│   └── mr_euroc.yaml            (legacy SuperPoint config)
│
├── configs/relocalization/
│   ├── reloc_euroc.yaml         (TODO: reloc_euroc_xfeat.yaml when needed)
│   └── reloc_tartanair.yaml
│
├── launch/visual_odometry/
│   ├── vo_euroc_xfeat_lighterglue.launch.py   ⭐ HEADLINE launch
│   ├── vo_euroc_xfeat.launch.py
│   ├── vo_euroc.launch.py        (SuperPoint baseline)
│   └── ...
│
├── launch/map_refinement/
│   ├── mr_euroc_xfeat.launch.py
│   └── mr_euroc.launch.py
│
├── output/                      (gitignored: *.engine, *_trt10.onnx)
│   ├── xfeat.onnx               2.7 MB (tracked)
│   ├── xfeat_trt10.onnx         (regenerable)
│   ├── xfeat.engine             7.6 MB (regenerable)
│   ├── lighterglue.pt           4.5 MB (tracked, static N=512)
│   ├── superpoint_v1_sim_int32.onnx     (legacy SP baseline)
│   ├── superpoint_lightglue.onnx        (legacy SP baseline)
│   ├── plnet_s0.onnx, plnet_s1.onnx     (legacy, not used in XFeat mode)
│   └── benchmarks_xfeat.md      Full benchmark log
│
├── voc/
│   ├── point_voc_L4_xfeat.bin   ⭐ 64-dim Apache-2.0 vocab (tracked)
│   └── point_voc_L4.bin         (SuperPoint-derived, tainted, kept for A/B)
│
├── rviz/
│   └── vo_jazzy.rviz            Single RViz2 config used by all launch files
│
├── 3rdparty/
│   ├── tensorrtbuffer/          Name-keyed TRT 10 BufferManager
│   └── DBoW2/                   Templated bag-of-words (unmodified upstream)
│
├── CMakeLists.txt               ament_cmake, project(air_slam_xfeat)
├── package.xml                  ament_cmake format 3
├── CLAUDE.md                    Orientation map for Claude Code
├── STATUS.md                    Measured numbers + future work
├── ARCHITECTURE.md              This file
├── README.md                    Upstream README (preserved)
├── README_JAZZY.md              jazzy-port era doc
└── README_JAZZY_XFEAT.md        XFeat delta vs jazzy-port
```

## Build / link graph

```
                  ┌────────────────────────────────┐
                  │  demo/visual_odometry.cpp      │ -> visual_odometry exe
                  │  demo/map_refinement.cpp       │ -> map_refinement exe
                  │  demo/relocalization.cpp       │ -> relocalization exe
                  │  demo/test_xfeat.cpp           │ -> test_xfeat exe
                  │  demo/test_feature.cpp         │ -> test_feature exe
                  │  demo/train_voc_xfeat.cpp      │ -> train_voc_xfeat exe
                  └─────────────────┬──────────────┘
                                    │ links to
                                    ▼
                  ┌────────────────────────────────┐
                  │  air_slam_lib (SHARED)         │
                  │  - src/ros_publisher.cc        │
                  │  - src/map_builder.cc          │
                  │  - src/map_refiner.cc          │
                  │  - src/map_user.cc             │
                  │  - src/g2o_optimization/*.cc   │
                  └─────────────────┬──────────────┘
                                    │ links to
                                    ▼
                  ┌────────────────────────────────┐
                  │  air_slam_core_lib (SHARED)    │
                  │  - src/xfeat.cpp + xfeat_postproc.cu
                  │  - src/super_point.cpp         │
                  │  - src/lighter_glue.cc         │
                  │  - src/light_glue.cpp, super_glue.cpp
                  │  - src/plnet.cpp               │
                  │  - src/feature_detector.cc     │
                  │  - src/point_matcher.cc        │
                  │  - src/camera.cc, dataset.cc, imu.cc
                  │  - src/frame.cc, mappoint.cc, mapline.cc
                  │  - src/bow/* (FSuperpoint, database)
                  └─────────────────┬──────────────┘
                                    │ links to
                                    ▼
   tensorrtbuffer + DBoW2 + nvinfer + nvonnxparser + CUDA::cudart
   + CUDA::cublas + TORCH_LIBRARIES + OpenCV + yaml-cpp + Boost
   + G2O + Gflags + Glog
```

The two-library split (`core` non-ROS, `lib` ROS-aware) is preserved
from the upstream jazzy-port. **XFeat-mode additions (xfeat,
xfeat_postproc, lighter_glue) all live in core** — libtorch has no
rclcpp coupling, so it's safe to expose at the core level.

## ROS 2 graph (VO at runtime)

```
   Image source                         Output consumers
   ────────────                         ─────────────────
                                        ┌─ rviz2  (vo_jazzy.rviz)
   /camera/.../left   ──┐               ├─ Nav2 (subscriber on
   /camera/.../right  ──┼──> visual_odometry ────┤   /AirSLAM/LatestOdometry)
   /camera/.../imu    ──┘                        ├─ tf2 (map → diff_bot)
                                                 ├─ logging / rosbag
                                                 └─ your code
   topics on stereo+IMU input            topics on output
   (when D455 path lands;                /AirSLAM/feature      (Image)
    today: Dataset reader                /AirSLAM/frame_pose   (PoseStamped)
    in demo/visual_odometry.cpp)         /AirSLAM/LatestOdometry (Odometry)
                                         /AirSLAM/keyframe     (Image)
                                         /AirSLAM/odometry     (Path)
                                         /AirSLAM/map          (MarkerArray)
                                         /AirSLAM/mapline      (MarkerArray, empty)
                                         /AirSLAM/reloc        (Image)
                                         /tf  (map → diff_bot, /world)
```

For the EuRoC offline benchmark mode, `demo/visual_odometry.cpp` reads
images from disk via `Dataset` and feeds them into MapBuilder serially.
For live D455 input, the cherry-pick from `master` branch (Future-work
item) would replace the Dataset reader with rclcpp subscribers wrapped
in `message_filters::ApproximateTimeSynchronizer`.
