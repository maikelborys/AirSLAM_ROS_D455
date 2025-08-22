# ROS2 Bag to EuRoC Format Conversion Guide

This guide explains how to convert ROS2 bags from Intel RealSense D455 camera to EuRoC dataset format for use with AirSLAM.

## Overview

AirSLAM requires data in EuRoC dataset format, which consists of:
- Stereo images (left/right camera)
- IMU data
- Camera calibration parameters
- Proper directory structure

## Prerequisites

- ROS2 Humble installed
- Intel RealSense D455 camera
- Python3 with required packages (cv2, numpy, rclpy, cv_bridge)

## Step 1: Record ROS2 Bag with D455

### 1.1 Start D455 with Clean IR Images (No Projector)

```bash
# Run camera in STEREO mode with IR projector DISABLED
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

### 1.2 Disable IR Projector (if needed)

```bash
# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Disable IR emitter
ros2 param set /camera/camera depth_module.emitter_enabled 0
ros2 param set /camera/camera depth_module.emitter_always_on false
```

### 1.3 Record Bag

```bash
# Record stereo + IMU data (clean IR images)
rosbag record -O d455_test.bag \
  /camera/camera/infra1/image_rect_raw \
  /camera/camera/infra2/image_rect_raw \
  /camera/camera/infra1/camera_info \
  /camera/camera/infra2/camera_info \
  /camera/camera/imu \
  --duration=60s
```

## Step 2: Create ROS2 Workspace for Conversion

### 2.1 Create Workspace Structure

```bash
cd ~
mkdir -p ros2_ws/src
cd ros2_ws/src
```

### 2.2 Create ROS2 Package

```bash
ros2 pkg create --build-type ament_python bag_converter --dependencies rclpy sensor_msgs cv_bridge
```

## Step 3: Create Conversion Script

### 3.1 Create the Converter Node

Create file: `~/ros2_ws/src/bag_converter/bag_converter/bag_to_euroc_converter.py`

```python
#!/usr/bin/env python3
"""
ROS2 subscriber to convert bag data to EuRoC format
This node subscribes to camera and IMU topics and saves data in EuRoC format
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image, CameraInfo, Imu
from cv_bridge import CvBridge
import cv2
import os
import csv
import numpy as np

class BagToEuRoCConverter(Node):
    def __init__(self, output_dir):
        super().__init__('bag_to_euroc_converter')
        
        self.output_dir = output_dir
        self.bridge = CvBridge()
        
        # Create output directory structure
        self.create_euroc_structure()
        
        # Initialize counters
        self.left_count = 0
        self.right_count = 0
        self.imu_count = 0
        
        # Open IMU CSV file
        self.imu_csv_path = os.path.join(output_dir, "mav0", "imu0", "data.csv")
        self.imu_file = open(self.imu_csv_path, 'w', newline='')
        self.imu_writer = csv.writer(self.imu_file)
        self.imu_writer.writerow(['#timestamp [ns]', 'w_RS_S_x [rad s^-1]', 'w_RS_S_y [rad s^-1]', 'w_RS_S_z [rad s^-1]', 
                                 'a_RS_S_x [m s^-2]', 'a_RS_S_y [m s^-2]', 'a_RS_S_z [m s^-2]'])
        
        # Create QoS profile for IMU (to fix compatibility issues)
        imu_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=100
        )
        
        # Create subscribers
        self.left_image_sub = self.create_subscription(
            Image, '/camera/camera/infra1/image_rect_raw', self.left_image_callback, 10)
        self.right_image_sub = self.create_subscription(
            Image, '/camera/camera/infra2/image_rect_raw', self.right_image_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/camera/camera/imu', self.imu_callback, imu_qos)
        
        self.get_logger().info(f'Converter started, saving to {output_dir}')
        
        # Timer to print progress
        self.timer = self.create_timer(5.0, self.print_progress)
    
    def create_euroc_structure(self):
        """Create EuRoC dataset directory structure"""
        os.makedirs(self.output_dir, exist_ok=True)
        os.makedirs(os.path.join(self.output_dir, "mav0"), exist_ok=True)
        os.makedirs(os.path.join(self.output_dir, "mav0", "cam0", "data"), exist_ok=True)
        os.makedirs(os.path.join(self.output_dir, "mav0", "cam1", "data"), exist_ok=True)
        os.makedirs(os.path.join(self.output_dir, "mav0", "imu0"), exist_ok=True)
        
        # Create body.yaml file
        body_yaml = """#body_T_cam0
body_T_cam0:
  rows: 4
  cols: 4
  data: [0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
         0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
         -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
         0.0, 0.0, 0.0, 1.0]
"""
        with open(os.path.join(self.output_dir, "mav0", "body.yaml"), 'w') as f:
            f.write(body_yaml)
        
        # Create IMU sensor.yaml file
        imu_sensor_yaml = """#Default imu sensor yaml file
sensor_type: imu
comment: RealSense D455 IMU

# Sensor extrinsics wrt. the body-frame.
T_BS:
  cols: 4
  rows: 4
  data: [1.0, 0.0, 0.0, 0.0,
         0.0, 1.0, 0.0, 0.0,
         0.0, 0.0, 1.0, 0.0,
         0.0, 0.0, 0.0, 1.0]
rate_hz: 200

# inertial sensor noise model parameters (static)
gyroscope_noise_density: 1.6968e-04     # [ rad / s / sqrt(Hz) ]   ( gyro "white noise" )
gyroscope_random_walk: 1.9393e-05       # [ rad / s^2 / sqrt(Hz) ] ( gyro bias diffusion )
accelerometer_noise_density: 2.0000e-3  # [ m / s^2 / sqrt(Hz) ]   ( accel "white noise" )
accelerometer_random_walk: 3.0000e-3    # [ m / s^3 / sqrt(Hz) ].  ( accel bias diffusion )
"""
        with open(os.path.join(self.output_dir, "mav0", "imu0", "sensor.yaml"), 'w') as f:
            f.write(imu_sensor_yaml)
    
    def left_image_callback(self, msg):
        """Save left camera images"""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "mono8")
            
            # Create filename from timestamp
            timestamp = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
            filename = f"{timestamp}.png"
            
            # Save image
            image_path = os.path.join(self.output_dir, "mav0", "cam0", "data", filename)
            cv2.imwrite(image_path, cv_image)
            
            self.left_count += 1
            
        except Exception as e:
            self.get_logger().error(f'Error saving left image: {e}')
    
    def right_image_callback(self, msg):
        """Save right camera images"""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "mono8")
            
            # Create filename from timestamp
            timestamp = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
            filename = f"{timestamp}.png"
            
            # Save image
            image_path = os.path.join(self.output_dir, "mav0", "cam1", "data", filename)
            cv2.imwrite(image_path, cv_image)
            
            self.right_count += 1
            
        except Exception as e:
            self.get_logger().error(f'Error saving right image: {e}')
    
    def imu_callback(self, msg):
        """Save IMU data"""
        try:
            # Create timestamp
            timestamp = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
            
            # Write IMU data to CSV
            self.imu_writer.writerow([
                timestamp,
                msg.angular_velocity.x,
                msg.angular_velocity.y, 
                msg.angular_velocity.z,
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z
            ])
            
            # Flush to ensure data is written
            self.imu_file.flush()
            
            self.imu_count += 1
            
        except Exception as e:
            self.get_logger().error(f'Error saving IMU data: {e}')
    
    def print_progress(self):
        """Print conversion progress"""
        self.get_logger().info(f'Progress - Left: {self.left_count}, Right: {self.right_count}, IMU: {self.imu_count}')
    
    def __del__(self):
        """Cleanup"""
        if hasattr(self, 'imu_file') and self.imu_file:
            self.imu_file.close()

def main():
    import sys
    
    if len(sys.argv) != 2:
        print("Usage: ros2 run bag_converter bag_to_euroc_converter <output_dir>")
        sys.exit(1)
    
    output_dir = sys.argv[1]
    
    rclpy.init()
    converter = BagToEuRoCConverter(output_dir)
    
    try:
        rclpy.spin(converter)
    except KeyboardInterrupt:
        print(f"\nConversion stopped. Final count:")
        print(f"Left images: {converter.left_count}")
        print(f"Right images: {converter.right_count}")
        print(f"IMU samples: {converter.imu_count}")
    finally:
        converter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 3.2 Update setup.py

Edit `~/ros2_ws/src/bag_converter/setup.py`:

```python
    entry_points={
        'console_scripts': [
            'bag_to_euroc_converter = bag_converter.bag_to_euroc_converter:main',
        ],
    },
```

## Step 4: Build and Run Conversion

### 4.1 Build ROS2 Workspace

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build
```

### 4.2 Create Conversion Script

Create file: `~/ros2_ws/convert_d455_bag.sh`

```bash
#!/bin/bash
"""
Convert D455 ROS2 bag to EuRoC format
Usage: ./convert_d455_bag.sh <bag_path> <output_dir>
"""

if [ $# -ne 2 ]; then
    echo "Usage: $0 <bag_path> <output_dir>"
    echo "Example: $0 /home/robot/datasets/bag_d455_1 /home/robot/datasets/d455_euroc_dataset"
    exit 1
fi

BAG_PATH=$1
OUTPUT_DIR=$2

echo "Converting $BAG_PATH to EuRoC format in $OUTPUT_DIR"

# Check if bag exists
if [ ! -d "$BAG_PATH" ]; then
    echo "Error: Bag directory $BAG_PATH not found"
    exit 1
fi

# Source ROS2 and our workspace
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Create output directory
mkdir -p "$OUTPUT_DIR"

echo "Starting converter node..."
# Start the converter node in the background
ros2 run bag_converter bag_to_euroc_converter "$OUTPUT_DIR" &
CONVERTER_PID=$!

# Wait a bit for the converter to start
sleep 3

echo "Playing bag file..."
# Play the bag at slower rate for better processing
ros2 bag play "$BAG_PATH" --rate 1.0

# Wait a bit more for final messages to be processed
echo "Waiting for final messages to be processed..."
sleep 5

# Kill the converter
echo "Stopping converter..."
kill $CONVERTER_PID 2>/dev/null
wait $CONVERTER_PID 2>/dev/null

echo "Conversion complete!"
echo "Dataset saved in: $OUTPUT_DIR"

# Show final statistics
echo ""
echo "Dataset contents:"
LEFT_IMAGES=$(find "$OUTPUT_DIR/mav0/cam0/data" -name "*.png" 2>/dev/null | wc -l)
RIGHT_IMAGES=$(find "$OUTPUT_DIR/mav0/cam1/data" -name "*.png" 2>/dev/null | wc -l)
IMU_LINES=$(wc -l < "$OUTPUT_DIR/mav0/imu0/data.csv" 2>/dev/null || echo "1")

echo "Left images: $LEFT_IMAGES"
echo "Right images: $RIGHT_IMAGES" 
echo "IMU samples: $((IMU_LINES - 1))"  # Subtract header

if [ "$LEFT_IMAGES" -gt 0 ] && [ "$RIGHT_IMAGES" -gt 0 ]; then
    echo ""
    echo "✅ Conversion successful!"
    echo ""
    echo "To run AirSLAM with this dataset:"
    echo "1. cd ~/catkin_ws"
    echo "2. source devel/setup.bash"
    echo "3. roslaunch air_slam vo_d455_dataset.launch"
else
    echo ""
    echo "❌ Conversion failed - no images found"
fi
```

### 4.3 Make Script Executable and Run

```bash
chmod +x ~/ros2_ws/convert_d455_bag.sh
cd ~/ros2_ws
./convert_d455_bag.sh /home/robot/datasets/bag_d455_1 /home/robot/datasets/d455_euroc_dataset
```

## Step 5: Configure AirSLAM for D455

### 5.1 Update Camera Configuration

**Key Changes Made to `catkin_ws/src/AirSLAM/configs/camera/realsense_848_480.yaml`:**

1. **Enable IMU**: Changed `use_imu: 0` to `use_imu: 1`
2. **Updated Intrinsics**: Replaced default values with actual D455 calibration:
   - **fx, fy**: `426.1531982421875` (focal length)
   - **cx, cy**: `423.66717529296875, 240.55062866210938` (principal point)
3. **Stereo Baseline**: Set to `0.05` meters (50mm) for D455
4. **Distortion**: Set to `0.0` since we use rectified images

```yaml
%YAML:1.0

image_height: 480
image_width: 848
use_imu: 1  # ENABLED for D455

depth_lower_thr: 0.1
depth_upper_thr: 10.0
max_y_diff: 2

# Calibration - UPDATED with actual D455 parameters
distortion_type: 0  # 0 for undistorted inputs
cam0:
  intrinsics: [426.1531982421875, 426.1531982421875, 423.66717529296875, 240.55062866210938] # fx, fy, cx, cy
  distortion_coeffs: [0.0, 0.0, 0.0, 0.0, 0]
  T_type: 0           
  T: 
  - [1.0, 0.0, 0.0, 0.0]
  - [0.0, 1.0, 0.0, 0.0]
  - [0.0, 0.0, 1.0, 0.0]
  - [0.0, 0.0, 0.0, 1.0]
cam1:
  intrinsics: [426.1531982421875, 426.1531982421875, 423.66717529296875, 240.55062866210938] # fx, fy, cx, cy
  distortion_coeffs: [0.0, 0.0, 0.0, 0.0, 0]
  T_type: 0           
  T: 
  - [1.0, 0.0, 0.0, 0.05]  # 50mm baseline for D455
  - [0.0, 1.0, 0.0, 0.0]
  - [0.0, 0.0, 1.0, 0.0]
  - [0.0, 0.0, 0.0, 1.0]
```

**How we obtained these parameters:**
```bash
# Get actual D455 camera parameters
rostopic echo /camera/camera/infra1/camera_info -n 1
rostopic echo /camera/camera/infra2/camera_info -n 1
```

### 5.2 Create Visual Odometry Config

**Created `catkin_ws/src/AirSLAM/configs/visual_odometry/vo_realsense.yaml`:**

**Key Configuration Changes for D455:**
1. **Image Resolution**: Set to `848x480` (D455 infrared resolution)
2. **Feature Detection**: Using SuperPoint + LightGlue (modern, efficient)
3. **Optimized Parameters**: Adjusted for D455 stereo baseline and image quality
4. **ROS Topics**: All AirSLAM output topics configured

```yaml
plnet:
  use_superpoint: 1          # Modern feature detector
  max_keypoints: 400         # Good for 848x480 resolution
  keypoint_threshold: 0.004  # Balanced sensitivity
  remove_borders: 4          # Remove edge features
  line_threshold: 0.75       # Line feature detection
  line_length_threshold: 50  # Minimum line length

point_matcher:
  matcher: 0                 # 0 for lightglue (faster than superglue)
  image_width: 848           # D455 infrared width
  image_height: 480          # D455 infrared height
  onnx_file: "superpoint_lightglue.onnx"
  engine_file: "superpoint_lightglue.engine"

keyframe:
  min_init_stereo_feature: 90  # Good for D455 stereo
  lost_num_match: 10
  min_num_match: 30
  max_num_match: 80
  tracking_point_rate: 0.65  
  tracking_parallax_rate: 0.1

optimization:
  tracking:
    mono_point: 50
    stereo_point: 75         # Higher weight for stereo (D455 strength)
    mono_line: 50
    stereo_line: 75
    rate: 0.5
  backend:
    mono_point: 50
    stereo_point: 75
    mono_line: 50
    stereo_line: 75
    rate: 0.5

ros_publisher:
  feature: 1
  feature_topic: "/AirSLAM/feature"
  frame_pose: 1
  frame_pose_topic: "/AirSLAM/frame_pose"
  frame_odometry_topic: "/AirSLAM/LatestOdometry"
  keyframe: 1
  keyframe_topic: "/AirSLAM/keyframe"
  path_topic: "/AirSLAM/odometry"
  map: 1
  map_topic: "/AirSLAM/map"
  mapline: 1
  mapline_topic: "/AirSLAM/mapline"
  reloc: 0
  reloc_topic: "/AirSLAM/reloc"
```

### 5.3 Create Launch File

**Created `catkin_ws/src/AirSLAM/launch/visual_odometry/vo_d455_dataset.launch`:**

**Key Launch File Configuration:**
1. **Dataset Path**: Points to converted EuRoC dataset
2. **Camera Config**: Uses updated D455 calibration
3. **VO Config**: Uses optimized D455 parameters
4. **Output Directory**: Saves results to debug folder

```xml
<launch>
  <arg name="config_path" default = "$(find air_slam)/configs/visual_odometry/vo_realsense.yaml" />
  <arg name="dataroot" default = "/home/robot/datasets/d455_euroc_dataset/mav0" />
  <arg name="camera_config_path" default = "$(find air_slam)/configs/camera/realsense_848_480.yaml" />
  <arg name="model_dir" default = "$(find air_slam)/output" />
  <arg name="saving_dir" default = "$(find air_slam)/debug" />

  <node name="visual_odometry" pkg="air_slam" type="visual_odometry" output="screen">
    <param name="config_path" type="string" value="$(arg config_path)" />
    <param name="dataroot" type="string" value="$(arg dataroot)" />
    <param name="camera_config_path" type="string" value="$(arg camera_config_path)" />
    <param name="model_dir" type="string" value="$(arg model_dir)" />
    <param name="saving_dir" type="string" value="$(arg saving_dir)" />
  </node>

  <arg name="visualization" default="true" />
  <group if="$(arg visualization)">
    <node name="rviz" pkg="rviz" type="rviz" args="-d $(find air_slam)/rviz/vo.rviz" output="screen" />
  </group>    
</launch>
```

## Step 6: Run AirSLAM

### 6.1 Build AirSLAM Workspace

```bash
cd ~/catkin_ws
source /home/robot/ros_catkin_ws/install_isolated/setup.bash
catkin_make
source devel/setup.bash
```

### 6.2 Run Visual Odometry

```bash
roslaunch air_slam vo_d455_dataset.launch
```

### 6.3 Run Map Refinement (after VO completes)

```bash
roslaunch air_slam mr_d455_dataset.launch
```

## Troubleshooting

### QoS Compatibility Issues
**Problem**: IMU data not being received due to ROS2 QoS profile mismatch
**Solution**: Configured QoS profile with BEST_EFFORT reliability:
```python
imu_qos = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=100
)
```

### Missing Images
**Problem**: IR projector dots interfering with feature detection
**Solution**: Disable IR projector completely:
```bash
ros2 param set /camera/camera depth_module.emitter_enabled 0
ros2 param set /camera/camera depth_module.emitter_always_on false
```

### Build Errors
**Problem**: Missing dependencies or compilation issues
**Solution**: Ensure all dependencies are installed:
- OpenCV 4.7+
- CUDA 12.1
- ROS2 Humble
- Python packages: cv_bridge, numpy

### ROS1/ROS2 Environment Conflicts
**Problem**: Environment mixing causing import errors
**Solution**: Use proper environment activation:
```bash
# For ROS1 (AirSLAM)
source /home/robot/ros_catkin_ws/install_isolated/setup.bash
export ROS_DISTRO=noetic

# For ROS2 (bag conversion)
source /opt/ros/humble/setup.bash
export ROS_DISTRO=humble
```

## Expected Output

After successful conversion, you should have:
- **Left images**: ~3000+ PNG files
- **Right images**: ~3000+ PNG files  
- **IMU data**: ~20,000+ samples in CSV format
- **EuRoC structure**: Proper directory layout with calibration files

## Dataset Statistics Example

```
Dataset contents:
Left images: 3080
Right images: 3079
IMU samples: 20790
```

## Conversion Process Summary

### What the Converter Does:
1. **Subscribes to ROS2 Topics**: 
   - `/camera/camera/infra1/image_rect_raw` (left camera)
   - `/camera/camera/infra2/image_rect_raw` (right camera)
   - `/camera/camera/imu` (IMU data)

2. **Creates EuRoC Structure**:
   ```
   d455_euroc_dataset/
   └── mav0/
       ├── body.yaml              # Camera-IMU transformation
       ├── cam0/
       │   └── data/              # Left images (timestamp.png)
       ├── cam1/
       │   └── data/              # Right images (timestamp.png)
       └── imu0/
           ├── data.csv           # IMU data in EuRoC format
           └── sensor.yaml        # IMU calibration
   ```

3. **Converts Data Formats**:
   - **Images**: ROS2 Image → OpenCV → PNG files
   - **IMU**: ROS2 Imu → CSV with EuRoC headers
   - **Timestamps**: ROS2 time → Nanosecond timestamps

### Key Technical Details:
- **QoS Profile**: BEST_EFFORT for IMU compatibility
- **Image Format**: Mono8 (grayscale infrared)
- **Timestamp Format**: Nanoseconds (EuRoC standard)
- **IMU Format**: 7 columns (timestamp + 6 IMU values)

This dataset can then be used with AirSLAM for visual odometry, map refinement, and relocalization.
