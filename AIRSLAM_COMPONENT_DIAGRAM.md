# 🏗️ AirSLAM Component Architecture Diagram

## 📊 **High-Level System Architecture**

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              AIRSLAM FRAMEWORK                              │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐         │
│  │   INPUT LAYER   │    │  PROCESSING     │    │   OUTPUT LAYER  │         │
│  │                 │    │     LAYER       │    │                 │         │
│  │ • Camera Data   │───▶│ • Feature Ext.  │───▶│ • Pose Output   │         │
│  │ • IMU Data      │    │ • Tracking      │    │ • Map Data      │         │
│  │ • ROS Topics    │    │ • Optimization  │    │ • Visualization │         │
│  └─────────────────┘    └─────────────────┘    └─────────────────┘         │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

## 🔧 **Core Component Relationships**

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              MAP BUILDER                                    │
│  ┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐         │
│  │   InputData     │    │  TrackingData   │    │     Frame       │         │
│  │                 │    │                 │    │                 │         │
│  │ • Stereo Images │───▶│ • Frame Pair    │───▶│ • Pose (4x4)    │         │
│  │ • IMU Batch     │    │ • Matches       │    │ • Features      │         │
│  │ • Timestamp     │    │ • Frame Type    │    │ • Keypoints     │         │
│  └─────────────────┘    └─────────────────┘    └─────────────────┘         │
│           │                       │                       │                 │
│           ▼                       ▼                       ▼                 │
│  ┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐         │
│  │ FeatureDetector │    │  PointMatcher   │    │      Map        │         │
│  │                 │    │                 │    │                 │         │
│  │ • SuperPoint    │    │ • LightGlue     │    │ • Keyframes     │         │
│  │ • PLNet         │    │ • SuperGlue     │    │ • Mappoints     │         │
│  │ • Junctions     │    │ • Matching      │    │ • Maplines      │         │
│  └─────────────────┘    └─────────────────┘    └─────────────────┘         │
└─────────────────────────────────────────────────────────────────────────────┘
```

## 🧠 **Algorithm Pipeline**

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              PROCESSING PIPELINE                            │
│                                                                             │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐       │
│  │   INPUT     │  │  FEATURE    │  │  TRACKING   │  │   OUTPUT    │       │
│  │             │  │ EXTRACTION  │  │             │  │             │       │
│  │ • Images    │─▶│ • SuperPoint│─▶│ • IMU Pre-  │─▶│ • Pose      │       │
│  │ • IMU       │  │ • Line Det. │  │   Integration│  │ • Map       │       │
│  │ • Timestamp │  │ • Junction  │  │ • Matching  │  │ • Trajectory│       │
│  └─────────────┘  └─────────────┘  └─────────────┘  └─────────────┘       │
│       │                   │                   │                   │       │
│       ▼                   ▼                   ▼                   ▼       │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐       │
│  │ Data Buffer │  │ Frame Queue │  │ Tracking    │  │ ROS Topics  │       │
│  │ (Thread-    │  │ (Thread-    │  │ Buffer      │  │ • /pose     │       │
│  │  Safe)      │  │  Safe)      │  │ (Thread-    │  │ • /map      │       │
│  └─────────────┘  └─────────────┘  │  Safe)      │  │ • /features │       │
│                                     └─────────────┘  └─────────────┘       │
└─────────────────────────────────────────────────────────────────────────────┘
```

## 🎯 **Key Data Structures**

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              DATA STRUCTURES                                │
│                                                                             │
│  ┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐         │
│  │     FRAME       │    │    MAPPOINT     │    │    MAPLINE      │         │
│  │                 │    │                 │    │                 │         │
│  │ • frame_id      │    │ • mappoint_id   │    │ • mapline_id    │         │
│  │ • timestamp     │    │ • position (3D) │    │ • endpoints (3D)│         │
│  │ • pose (4x4)    │    │ • descriptor    │    │ • descriptor    │         │
│  │ • features      │    │ • observations  │    │ • observations  │         │
│  │ • keypoints     │    │ • track_id      │    │ • track_id      │         │
│  │ • lines         │    │ • valid flag    │    │ • valid flag    │         │
│  │ • junctions     │    └─────────────────┘    └─────────────────┘         │
│  │ • velocity      │                                                       │
│  │ • bias          │                                                       │
│  └─────────────────┘                                                       │
│                                                                             │
│  ┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐         │
│  │      MAP        │    │   DATABASE      │    │   CAMERA        │         │
│  │                 │    │                 │    │                 │         │
│  │ • keyframes     │    │ • vocabulary    │    │ • intrinsics    │         │
│  │ • mappoints     │    │ • frame_bow     │    │ • distortion    │         │
│  │ • maplines      │    │ • word_features │    │ • stereo_baseline│        │
│  │ • covisibility  │    │ • scoring       │    │ • projection    │         │
│  │ • database      │    │ • query         │    │ • undistortion  │         │
│  └─────────────────┘    └─────────────────┘    └─────────────────┘         │
└─────────────────────────────────────────────────────────────────────────────┘
```

## 🔄 **Threading Architecture**

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              THREADING MODEL                                │
│                                                                             │
│  ┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐         │
│  │  INPUT THREAD   │    │ FEATURE THREAD  │    │ TRACKING THREAD │         │
│  │                 │    │                 │    │                 │         │
│  │ • ROS Callbacks │    │ • SuperPoint    │    │ • IMU Pre-int   │         │
│  │ • Data Buffering│    │ • Line Detection│    │ • Pose Est.     │         │
│  │ • Synchronization│   │ • Frame Creation│    │ • Keyframe Dec. │         │
│  │ • Rate Control  │    │ • Queue Push    │    │ • Map Update    │         │
│  └─────────────────┘    └─────────────────┘    └─────────────────┘         │
│           │                       │                       │                 │
│           ▼                       ▼                       ▼                 │
│  ┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐         │
│  │  DATA BUFFER    │    │ TRACKING BUFFER │    │  MAP THREAD     │         │
│  │                 │    │                 │    │                 │         │
│  │ • Thread-safe   │    │ • Thread-safe   │    │ • Optimization  │         │
│  │ • Queue-based   │    │ • Queue-based   │    │ • Loop Detection│         │
│  │ • Overflow Prot.│    │ • Overflow Prot.│    │ • Global BA     │         │
│  └─────────────────┘    └─────────────────┘    └─────────────────┘         │
└─────────────────────────────────────────────────────────────────────────────┘
```

## 🎯 **Configuration Hierarchy**

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              CONFIGURATION                                  │
│                                                                             │
│  ┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐         │
│  │  VO CONFIG      │    │ CAMERA CONFIG   │    │  MR CONFIG      │         │
│  │                 │    │                 │    │                 │         │
│  │ • plnet         │    │ • intrinsics    │    │ • min_inlier    │         │
│  │ • point_matcher │    │ • distortion    │    │ • pose_refinement│        │
│  │ • keyframe      │    │ • stereo_baseline│   │ • optimization  │         │
│  │ • optimization  │    │ • imu_params    │    │ • ros_publisher │         │
│  │ • ros_publisher │    │ • use_imu       │    └─────────────────┘         │
│  └─────────────────┘    └─────────────────┘                                 │
│           │                       │                                         │
│           ▼                       ▼                                         │
│  ┌─────────────────┐    ┌─────────────────┐                                 │
│  │  LAUNCH FILES   │    │  YAML PARSER    │                                 │
│  │                 │    │                 │                                 │
│  │ • vo_*.launch   │    │ • yaml-cpp      │                                 │
│  │ • mr_*.launch   │    │ • parameter     │                                 │
│  │ • reloc_*.launch│    │ • validation    │                                 │
│  └─────────────────┘    └─────────────────┘                                 │
└─────────────────────────────────────────────────────────────────────────────┘
```

## 🔧 **Dependencies & Libraries**

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              DEPENDENCIES                                   │
│                                                                             │
│  ┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐         │
│  │  CORE LIBS      │    │  DEEP LEARNING  │    │  OPTIMIZATION   │         │
│  │                 │    │                 │    │                 │         │
│  │ • OpenCV 4.7+   │    │ • SuperPoint    │    │ • g2o           │         │
│  │ • Eigen3        │    │ • LightGlue     │    │ • Ceres         │         │
│  │ • CUDA 12.1+    │    │ • SuperGlue     │    │ • DBoW2         │         │
│  │ • TensorRT      │    │ • PLNet         │    │ • Boost         │         │
│  │ • yaml-cpp      │    │ • ONNX Runtime  │    │ • Threading     │         │
│  └─────────────────┘    └─────────────────┘    └─────────────────┘         │
│           │                       │                       │                 │
│           ▼                       ▼                       ▼                 │
│  ┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐         │
│  │   ROS INTEG.    │    │  THIRD PARTY    │    │  UTILITIES      │         │
│  │                 │    │                 │    │                 │         │
│  │ • cv_bridge     │    │ • tensorrtbuffer│    │ • Timer         │         │
│  │ • message_filters│   │ • custom models │    │ • Debug         │         │
│  │ • geometry_msgs │    │ • calibration   │    │ • Utils         │         │
│  │ • sensor_msgs   │    │ • datasets      │    │ • Serialization│         │
│  └─────────────────┘    └─────────────────┘    └─────────────────┘         │
└─────────────────────────────────────────────────────────────────────────────┘
```

## 🎯 **Key Interfaces**

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              INTERFACES                                     │
│                                                                             │
│  ┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐         │
│  │  INPUT INTERF.  │    │ PROCESSING INT. │    │  OUTPUT INTER.  │         │
│  │                 │    │                 │    │                 │         │
│  │ • ROSDataset    │    │ • MapBuilder    │    │ • RosPublisher  │         │
│  │ • Dataset       │    │ • MapRefiner    │    │ • SaveMap       │         │
│  │ • InputData     │    │ • MapUser       │    │ • SaveTrajectory│         │
│  │ • Synchronization│   │ • FeatureDetector│   │ • Visualization │         │
│  └─────────────────┘    └─────────────────┘    └─────────────────┘         │
│           │                       │                       │                 │
│           ▼                       ▼                       ▼                 │
│  ┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐         │
│  │  CONFIG INTER.  │    │  OPTIMIZATION   │    │  UTILITY INTER. │         │
│  │                 │    │    INTERFACE    │    │                 │         │
│  │ • YAML Config   │    │ • g2o Factors   │    │ • Timer         │         │
│  │ • Parameter     │    │ • Constraints   │    │ • Debug         │         │
│  │ • Validation    │    │ • Vertices      │    │ • Utils         │         │
│  │ • Defaults      │    │ • Optimization  │    │ • Serialization│         │
│  └─────────────────┘    └─────────────────┘    └─────────────────┘         │
└─────────────────────────────────────────────────────────────────────────────┘
```

## 📊 **Performance Metrics**

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              PERFORMANCE                                    │
│                                                                             │
│  ┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐         │
│  │  TIMING (ms)    │    │  ACCURACY (%)   │    │  MEMORY (MB)    │         │
│  │                 │    │                 │    │                 │         │
│  │ • Feature Ext:  │    │ • Feature Match:│    │ • Frame: 1-5    │         │
│  │   10-30ms       │    │   80-95%        │    │ • Map: 10-100   │         │
│  │ • Matching:     │    │ • Pose Est:     │    │ • Features:     │         │
│  │   5-15ms        │    │   Sub-cm        │    │   50-200        │         │
│  │ • Pose Est:     │    │ • Loop Closure: │    │ • Total:        │         │
│  │   20-50ms       │    │   90-99%        │    │   200MB-2GB     │         │
│  │ • Optimization: │    │ • Relocalization│    │                 │         │
│  │   100-500ms     │    │   60-90%        │    │                 │         │
│  └─────────────────┘    └─────────────────┘    └─────────────────┘         │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

*This diagram provides a visual overview of the AirSLAM component architecture, showing the relationships between key modules, data flow, and system organization.*
