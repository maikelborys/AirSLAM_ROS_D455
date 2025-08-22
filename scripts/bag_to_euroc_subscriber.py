#!/usr/bin/env python3
"""
ROS2 subscriber to convert bag data to EuRoC format
This node subscribes to camera and IMU topics and saves data in EuRoC format
"""

import rclpy
from rclpy.node import Node
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
        self.imu_writer.writerow(['#timestamp [ns]', 'gyro_x [rad/s]', 'gyro_y [rad/s]', 'gyro_z [rad/s]', 
                                 'accel_x [m/s^2]', 'accel_y [m/s^2]', 'accel_z [m/s^2]'])
        
        # Create subscribers
        self.left_image_sub = self.create_subscription(
            Image, '/camera/camera/infra1/image_rect_raw', self.left_image_callback, 10)
        self.right_image_sub = self.create_subscription(
            Image, '/camera/camera/infra2/image_rect_raw', self.right_image_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/camera/camera/imu', self.imu_callback, 100)
        
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
        print("Usage: python3 bag_to_euroc_subscriber.py <output_dir>")
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


