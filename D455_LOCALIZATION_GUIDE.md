# 🎯 D455 Real-Time Localization Guide

Complete guide for using **AirSLAM** with Intel RealSense D455 for **real-time localization** against pre-built maps.

## 📋 **Overview**

**Localization** (also called relocalization) uses a **pre-built map** to determine the camera's position in real-time **without building a new map**. This is essential for:
- **Robot navigation** in known environments
- **AR/VR applications** with persistent maps
- **Multi-session SLAM** with map reuse
- **Loop closure detection** and recovery

## 🚀 **Quick Start**

### **Step 1: Build a Map First**
Before localization, you need a reference map from visual odometry:

```bash
# 1️⃣ Start D455 Camera (ROS2)
ros2env
ros2 run realsense2_camera realsense2_camera_node --ros-args \
  -p enable_infra1:=true -p enable_infra2:=true \
  -p enable_depth:=false -p enable_color:=false \
  -p enable_gyro:=true -p enable_accel:=true -p unite_imu_method:=2 \
  -p infra_width:=848 -p infra_height:=480 -p infra_fps:=30 \
  -p depth_module.emitter_enabled:=0 -p depth_module.emitter_always_on:=false

# 2️⃣ Bridge Topics (ROS2 → ROS1)  
ros1_bridge dynamic_bridge

# 3️⃣ Build Initial Map (ROS1)
rosenv
source devel/setup.bash
roslaunch air_slam vo_d455_live.launch     # Visual Odometry with IMU
# Move the camera around to build a good map, then Ctrl+C

# 4️⃣ Refine the Map (Optional but Recommended)
roslaunch air_slam mr_d455.launch          # Map Refinement with loop closures
```

### **Step 2: Real-Time Localization**
Now use the pre-built map for real-time localization:

```bash
# 1️⃣ Start D455 Camera (same as above)
ros2env
ros2 run realsense2_camera realsense2_camera_node --ros-args \
  -p enable_infra1:=true -p enable_infra2:=true \
  -p enable_depth:=false -p enable_color:=false \
  -p enable_gyro:=true -p enable_accel:=true -p unite_imu_method:=2 \
  -p infra_width:=848 -p infra_height:=480 -p infra_fps:=30 \
  -p depth_module.emitter_enabled:=0 -p depth_module.emitter_always_on:=false

# 2️⃣ Bridge Topics (same as above)
ros1_bridge dynamic_bridge

# 3️⃣ Run Real-Time Localization (ROS1)
rosenv
source devel/setup.bash
roslaunch air_slam reloc_d455.launch       # 🎯 LIVE LOCALIZATION!
```

---

## 📁 **Generated Files**

### **After Visual Odometry:**
```bash
~/catkin_ws/src/AirSLAM/debug/
├── AirSLAM_mapv0.bin     # Initial SLAM map
└── trajectory_v0.txt     # Initial trajectory
```

### **After Map Refinement:**
```bash
~/catkin_ws/src/AirSLAM/debug/
├── AirSLAM_mapv0.bin     # Original map (preserved)
├── trajectory_v0.txt     # Original trajectory (preserved)
├── AirSLAM_mapv1.bin     # ✅ REFINED map (used for localization)
└── trajectory_v1.txt     # ✅ REFINED trajectory
```

### **After Localization:**
```bash
~/catkin_ws/src/AirSLAM/debug/
└── relocalization_d455.txt    # ✅ Real-time localization results
```

---

## ⚙️ **Configuration Files**

### **🎯 Localization Config** (`configs/relocalization/reloc_d455.yaml`)
```yaml
min_inlier_num: 45              # Minimum feature matches for successful localization
pose_refinement: 1              # Enable pose optimization (more stable but slower)

plnet:
  max_keypoints: 400            # Match D455 VO settings for consistency
  keypoint_threshold: 0.004     # Same as D455 VO for feature consistency
  line_threshold: 0.75          # Tuned for D455 IR images

point_matcher:
  image_width: 848              # D455 IR resolution
  image_height: 480             # D455 IR resolution

ros_publisher:
  feature: 1                    # Show feature matches
  frame_pose: 1                 # Show real-time pose
  map: 1                        # Show loaded map
  reloc: 1                      # Show relocalization results
```

### **📷 Camera Config** (Uses same as VO: `configs/camera/realsense_848_480.yaml`)
- **IMU**: Not required for localization (uses visual features only)
- **Resolution**: 848x480 (matches map-building resolution)
- **Intrinsics**: Must match the camera used for map building

---

## 🎯 **How It Works**

### **1. Map Loading**
```cpp
MapUser map_user(configs, nh);
map_user.LoadMap("/path/to/debug");        // Loads AirSLAM_mapv1.bin
map_user.LoadVocabulary("/path/to/voc");   // Loads point_voc_L4.bin
```

### **2. Feature Matching**
- **Extract features** from live camera image using SuperPoint
- **Query database** for similar keyframes in the pre-built map
- **Match features** between live image and map keyframes
- **Filter matches** based on sharing words and scores

### **3. Pose Estimation**
- **PnP solver** estimates camera pose from 3D-2D correspondences
- **RANSAC** removes outliers and finds best pose
- **Pose refinement** (optional) optimizes the result
- **Success criteria**: Minimum 45 inlier matches

### **4. Real-Time Output**
- **ROS topics**: Live pose published to `/AirSLAM/frame_pose`
- **Visualization**: Feature matches and pose in RViz
- **Trajectory file**: All localization attempts saved

---

## 📊 **Performance & Tuning**

### **Expected Performance**
| Metric | **Value** | **Notes** |
|--------|-----------|-----------|
| **Processing Rate** | 10 Hz | Slower than VO (30 Hz) for stability |
| **Localization Time** | 50-200 ms | Depends on map size and features |
| **Success Rate** | 60-90% | Depends on map quality and scene overlap |
| **Memory Usage** | 1-3 GB | Depends on map size |

### **🎯 Tuning Parameters**

#### **For Higher Success Rate:**
```yaml
min_inlier_num: 30              # Lower threshold (vs 45 default)
max_keypoints: 600              # More features (vs 400 default)
keypoint_threshold: 0.003       # More sensitive detection
```

#### **For Faster Processing:**
```yaml
pose_refinement: 0              # Disable optimization
max_keypoints: 200              # Fewer features
keypoint_threshold: 0.006       # Higher threshold
```

#### **For Better Accuracy:**
```yaml
pose_refinement: 1              # Enable optimization (default)
min_inlier_num: 60              # Higher threshold
```

---

## 🛠️ **Troubleshooting**

### **❌ "No map found" Error**
```bash
# Check if map files exist:
ls -la ~/catkin_ws/src/AirSLAM/debug/
# Should see: AirSLAM_mapv1.bin (or AirSLAM_mapv0.bin)

# If missing, build a map first:
roslaunch air_slam vo_d455_live.launch
```

### **❌ "Low success rate" (<50%)**
**Possible causes:**
1. **Poor map quality**: Build map in better lighting, more features
2. **Different viewpoint**: Camera too far from original map-building trajectory  
3. **Scene changes**: Environment modified since map building
4. **Parameter mismatch**: Ensure same camera config as map building

**Solutions:**
```yaml
# Try lower thresholds:
min_inlier_num: 25
keypoint_threshold: 0.003

# Or rebuild map with more coverage:
roslaunch air_slam vo_d455_live.launch  # Move camera more thoroughly
```

### **❌ "Slow processing" (>500ms)**
```yaml
# Reduce computational load:
max_keypoints: 150
pose_refinement: 0
feature: 0        # Disable visualization
```

### **❌ "No camera data" Error**
```bash
# Check D455 is publishing:
rostopic list | grep camera
rostopic echo /camera/camera/infra1/image_rect_raw --max-count 1

# Check bridge is running:
ps aux | grep bridge
```

---

## 🎯 **Advanced Usage**

### **📍 Multi-Map Localization**
```bash
# Build multiple maps:
roslaunch air_slam vo_d455_live.launch    # Map 1 (move to different location)
mv debug/AirSLAM_mapv1.bin debug/map1.bin

roslaunch air_slam vo_d455_live.launch    # Map 2 (different area)  
mv debug/AirSLAM_mapv1.bin debug/map2.bin

# Localize against specific map:
roslaunch air_slam reloc_d455.launch map_root:=/path/to/map1/
```

### **🔄 Continuous Localization**
```bash
# Run localization in loop:
while true; do
  roslaunch air_slam reloc_d455.launch
  sleep 1
done
```

### **📊 Batch Evaluation**
```bash
# Record bag first:
rosbag record /camera/camera/infra1/image_rect_raw -O test_sequence.bag

# Then evaluate localization:
rosbag play test_sequence.bag --clock &
roslaunch air_slam reloc_d455.launch

# Check results:
cat debug/relocalization_d455.txt
```

---

## 🎯 **Integration with Navigation**

### **ROS Navigation Stack**
```bash
# Subscribe to localization pose:
rostopic echo /AirSLAM/frame_pose

# Convert to nav_msgs/Odometry:
# (Custom node needed to bridge AirSLAM pose to nav stack)
```

### **Custom Applications**
```cpp
// C++ subscriber example:
#include <geometry_msgs/PoseStamped.h>

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  // Use pose for robot control
  double x = msg->pose.position.x;
  double y = msg->pose.position.y;
  double z = msg->pose.position.z;
  
  // Your navigation logic here...
}

ros::Subscriber pose_sub = nh.subscribe("/AirSLAM/frame_pose", 1, poseCallback);
```

---

## 📋 **Summary**

D455 real-time localization provides:

✅ **Real-time pose estimation** at 10 Hz  
✅ **Pre-built map reuse** for known environments  
✅ **High accuracy** with pose refinement  
✅ **ROS integration** for navigation stacks  
✅ **Robust feature matching** with SuperPoint + LightGlue  
✅ **Comprehensive visualization** in RViz  

Perfect for **autonomous navigation**, **AR/VR applications**, and **multi-session SLAM**! 🚀

---
*Compatible with: ROS Noetic + Intel RealSense D455*  
*Requires: Pre-built AirSLAM map from visual odometry*
