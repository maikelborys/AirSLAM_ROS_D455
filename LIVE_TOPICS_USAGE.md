# AirSLAM Live Topics Usage Guide

This guide explains how to use AirSLAM with live ROS topics from Intel RealSense D455 camera instead of pre-recorded datasets.

## Overview

We've added live topic support to AirSLAM while preserving the original file-based functionality:

- **Original**: `visual_odometry` - reads from EuRoC dataset files
- **NEW**: `visual_odometry_live` - subscribes to live ROS topics

## Files Added/Modified

### New Files Created:
- `include/ros_dataset.h` - ROS topic-based dataset header
- `src/ros_dataset.cc` - ROS topic-based dataset implementation  
- `demo/visual_odometry_live.cpp` - Live topic visual odometry node
- `launch/visual_odometry/vo_d455_live.launch` - Live D455 launch file
- `launch/visual_odometry/vo_d455_complete.launch` - Complete setup launch file

### Modified Files:
- `CMakeLists.txt` - Added message_filters dependency and new executable

### Original Files (Preserved):
- All original files remain unchanged and functional
- Original `visual_odometry` node still works with datasets

## Setup Instructions

### Step 1: Build the Updated AirSLAM

```bash
cd ~/catkin_ws
source /home/robot/ros_catkin_ws/install_isolated/setup.bash
catkin_make
source devel/setup.bash
```

### Step 2: Start D455 Camera (ROS2)

In Terminal 1:
```bash
source /opt/ros/humble/setup.bash

# Start D455 with clean IR images (no projector)
ros2 run realsense2_camera realsense2_camera_node --ros-args \
  -p enable_infra1:=true \
  -p enable_infra2:=true \
  -p enable_depth:=false \
  -p enable_color:=false \
  -p enable_gyro:=true \
  -p enable_accel:=true \
  -p unite_imu_method:=2 \
  -p infra_width:=848 -p infra_height:=480 -p infra_fps:=30 \
  -p publish_tf:=true \
  -p enable_auto_exposure:=true \
  -p depth_module.emitter_enabled:=0 \
  -p depth_module.emitter_always_on:=false
```

### Step 3: Start ROS1/ROS2 Bridge

In Terminal 2:
```bash
source /home/robot/ros_catkin_ws/install_isolated/setup.bash  # ROS1
source /opt/ros/humble/setup.bash                            # ROS2
source ~/bridge_ws/install/setup.bash                        # Bridge workspace

# Start the bridge
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
```

### Step 4: Run AirSLAM Live

In Terminal 3:
```bash
cd ~/catkin_ws
source /home/robot/ros_catkin_ws/install_isolated/setup.bash
source devel/setup.bash

# Run live visual odometry
roslaunch air_slam vo_d455_live.launch
```

## Configuration

### Topic Names (Configurable)
The live node subscribes to these topics by default:
- Left camera: `/camera/camera/infra1/image_rect_raw`
- Right camera: `/camera/camera/infra2/image_rect_raw`  
- IMU data: `/camera/camera/imu`

To use different topic names:
```bash
roslaunch air_slam vo_d455_live.launch \
  left_image_topic:="/your/left/topic" \
  right_image_topic:="/your/right/topic" \
  imu_topic:="/your/imu/topic"
```

### Parameters

| Parameter | Description | Default |
|-----------|-------------|---------|
| `use_live_topics` | Enable topic mode (vs file mode) | `true` |
| `left_image_topic` | Left camera topic | `/camera/camera/infra1/image_rect_raw` |
| `right_image_topic` | Right camera topic | `/camera/camera/infra2/image_rect_raw` |
| `imu_topic` | IMU topic | `/camera/camera/imu` |

## Technical Details

### Stereo Synchronization
- Uses `message_filters` for stereo image synchronization
- Approximate time policy with 10ms tolerance
- Handles slight timestamp differences between left/right cameras

### IMU Integration
- IMU data is buffered independently 
- Associated with image frames based on timestamps
- IMU data between consecutive frames is grouped together

### Thread Safety
- All data buffers are mutex-protected
- Safe for concurrent ROS callbacks and processing

### Performance Considerations
- Buffer sizes limited to prevent memory issues (100 stereo pairs, 1000 IMU samples)
- Processing rate limited to 30 Hz to prevent overwhelming
- Queue-based processing ensures real-time performance

## Usage Examples

### Basic Live SLAM
```bash
roslaunch air_slam vo_d455_live.launch
```

### Live SLAM without RViz
```bash
roslaunch air_slam vo_d455_live.launch visualization:=false
```

### Custom Save Directory
```bash
roslaunch air_slam vo_d455_live.launch saving_dir:="/path/to/save"
```

### File-based Mode (Original)
```bash
# Still works with datasets
roslaunch air_slam vo_euroc.launch
```

## Troubleshooting

### No Data Received
**Problem**: AirSLAM starts but receives no camera data
**Check**: 
- Bridge is running and topics are available: `rostopic list`
- Camera is publishing: `rostopic echo /camera/camera/infra1/image_rect_raw`
- Topic names match in launch file

### Synchronization Issues  
**Problem**: Only receiving left or right images
**Check**:
- Both cameras publishing at same rate
- Timestamps are reasonable (not too different)
- No network delays in topics

### High CPU Usage
**Problem**: System performance issues
**Solution**:
- Reduce camera framerate: `infra_fps:=15`
- Increase processing rate limit in code
- Close RViz if not needed

### Bridge Connection Issues
**Problem**: Topics not bridging from ROS2 to ROS1
**Solution**:
- Ensure both ROS1 and ROS2 environments are sourced
- Check bridge workspace is built correctly
- Restart bridge if topics change

## Comparison: File vs Live Mode

| Aspect | File Mode (Original) | Live Mode (New) |
|--------|---------------------|-----------------|
| Input | EuRoC dataset files | ROS topics |
| Timing | Fixed dataset order | Real-time streams |
| IMU | Pre-synchronized | Real-time sync |
| Performance | Deterministic | Variable (real-time) |
| Debugging | Repeatable | Live data only |
| Setup | Simple | Requires camera+bridge |

## Next Steps

After live visual odometry works:

1. **Map Refinement**: Create live version of map refinement
2. **Relocalization**: Add live relocalization capability  
3. **ROS2 Direct**: Bypass bridge with native ROS2 support
4. **Multi-camera**: Extend to multiple camera systems

## Backup/Restore

All original functionality is preserved. To revert:
1. Use original launch files (`vo_euroc.launch`)
2. Original `visual_odometry` executable unchanged
3. All modifications are additive (commented, not deleted)

The live topic system is designed to coexist with the original file-based system.
