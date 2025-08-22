#!/usr/bin/env python3
"""
Convert rosbag to EuRoC dataset format for AirSLAM
Usage: python3 bag_to_euroc.py <bag_file> <output_dir>
"""

import os
import sys
import cv2
import numpy as np
import rosbag
from sensor_msgs.msg import Image, CameraInfo, Imu
from cv_bridge import CvBridge
import argparse
from datetime import datetime
import csv

def create_euroc_structure(output_dir):
    """Create EuRoC dataset directory structure"""
    os.makedirs(output_dir, exist_ok=True)
    os.makedirs(os.path.join(output_dir, "mav0"), exist_ok=True)
    os.makedirs(os.path.join(output_dir, "mav0", "cam0", "data"), exist_ok=True)
    os.makedirs(os.path.join(output_dir, "mav0", "cam1", "data"), exist_ok=True)
    os.makedirs(os.path.join(output_dir, "mav0", "imu0"), exist_ok=True)
    
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
    with open(os.path.join(output_dir, "mav0", "body.yaml"), 'w') as f:
        f.write(body_yaml)

def convert_bag_to_euroc(bag_path, output_dir):
    """Convert rosbag to EuRoC format"""
    print(f"Converting {bag_path} to EuRoC format in {output_dir}")
    
    # Create directory structure
    create_euroc_structure(output_dir)
    
    # Initialize CV bridge
    bridge = CvBridge()
    
    # Open rosbag
    bag = rosbag.Bag(bag_path)
    
    # Initialize data storage
    left_images = []
    right_images = []
    imu_data = []
    left_camera_info = None
    right_camera_info = None
    
    print("Reading rosbag...")
    
    # Read all messages
    for topic, msg, t in bag.read_messages():
        timestamp = t.to_nsec()  # Convert to nanoseconds
        
        if topic == "/camera/camera/infra1/image_rect_raw":
            # Convert ROS image to OpenCV
            cv_image = bridge.imgmsg_to_cv2(msg, "mono8")
            left_images.append((timestamp, cv_image))
            
        elif topic == "/camera/camera/infra2/image_rect_raw":
            cv_image = bridge.imgmsg_to_cv2(msg, "mono8")
            right_images.append((timestamp, cv_image))
            
        elif topic == "/camera/camera/infra1/camera_info":
            left_camera_info = msg
            
        elif topic == "/camera/camera/infra2/camera_info":
            right_camera_info = msg
            
        elif topic == "/camera/camera/imu":
            # Extract IMU data
            imu_data.append({
                'timestamp': timestamp,
                'gyro_x': msg.angular_velocity.x,
                'gyro_y': msg.angular_velocity.y,
                'gyro_z': msg.angular_velocity.z,
                'accel_x': msg.linear_acceleration.x,
                'accel_y': msg.linear_acceleration.y,
                'accel_z': msg.linear_acceleration.z
            })
    
    bag.close()
    
    print(f"Found {len(left_images)} left images, {len(right_images)} right images, {len(imu_data)} IMU samples")
    
    # Sort by timestamp
    left_images.sort(key=lambda x: x[0])
    right_images.sort(key=lambda x: x[0])
    imu_data.sort(key=lambda x: x['timestamp'])
    
    # Save images
    print("Saving images...")
    for i, (timestamp, image) in enumerate(left_images):
        filename = f"{timestamp}.png"
        cv2.imwrite(os.path.join(output_dir, "mav0", "cam0", "data", filename), image)
    
    for i, (timestamp, image) in enumerate(right_images):
        filename = f"{timestamp}.png"
        cv2.imwrite(os.path.join(output_dir, "mav0", "cam1", "data", filename), image)
    
    # Save IMU data
    print("Saving IMU data...")
    imu_csv_path = os.path.join(output_dir, "mav0", "imu0", "data.csv")
    with open(imu_csv_path, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['#timestamp [ns]', 'gyro_x [rad/s]', 'gyro_y [rad/s]', 'gyro_z [rad/s]', 
                        'accel_x [m/s^2]', 'accel_y [m/s^2]', 'accel_z [m/s^2]'])
        for imu in imu_data:
            writer.writerow([
                imu['timestamp'],
                imu['gyro_x'], imu['gyro_y'], imu['gyro_z'],
                imu['accel_x'], imu['accel_y'], imu['accel_z']
            ])
    
    # Create camera info files
    if left_camera_info:
        create_camera_info_file(left_camera_info, os.path.join(output_dir, "mav0", "cam0", "sensor.yaml"))
    if right_camera_info:
        create_camera_info_file(right_camera_info, os.path.join(output_dir, "mav0", "cam1", "sensor.yaml"))
    
    print(f"Conversion complete! Dataset saved to {output_dir}")
    print(f"Left images: {len(left_images)}")
    print(f"Right images: {len(right_images)}")
    print(f"IMU samples: {len(imu_data)}")

def create_camera_info_file(camera_info, output_path):
    """Create camera sensor.yaml file"""
    yaml_content = f"""#cam0
cam0:
  T_cam_imu:
    rows: 4
    cols: 4
    data: [0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
           0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
           -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
           0.0, 0.0, 0.0, 1.0]
  rate_hz: 30
  resolution: [{camera_info.width}, {camera_info.height}]
  camera_model: pinhole
  intrinsics: [{camera_info.K[0]}, {camera_info.K[4]}, {camera_info.K[2]}, {camera_info.K[5]}]
  distortion_model: plumb_bob
  distortion_coeffs: [{camera_info.D[0]}, {camera_info.D[1]}, {camera_info.D[2]}, {camera_info.D[3]}, {camera_info.D[4]}]
"""
    with open(output_path, 'w') as f:
        f.write(yaml_content)

def main():
    parser = argparse.ArgumentParser(description='Convert rosbag to EuRoC dataset format')
    parser.add_argument('bag_file', help='Input rosbag file')
    parser.add_argument('output_dir', help='Output directory for EuRoC dataset')
    
    args = parser.parse_args()
    
    if not os.path.exists(args.bag_file):
        print(f"Error: Bag file {args.bag_file} not found")
        sys.exit(1)
    
    convert_bag_to_euroc(args.bag_file, args.output_dir)

if __name__ == "__main__":
    main()
