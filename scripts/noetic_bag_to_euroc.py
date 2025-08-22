#!/usr/bin/env python3
"""
ROS Noetic Bag to EuRoC Dataset Converter for D455

Converts ROS1 bag files containing D455 stereo images and IMU data
into EuRoC dataset format for AirSLAM processing.

Usage:
    python3 noetic_bag_to_euroc.py /path/to/input.bag /path/to/output_dataset

Author: Claude AI Assistant
Compatible with: ROS Noetic + Intel RealSense D455
"""

import rosbag
import rospy
import cv2
import numpy as np
import os
import csv
import argparse
import yaml
from pathlib import Path
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, Imu
import shutil

class NoeticBagToEuRoC:
    def __init__(self, bag_path, output_path):
        self.bag_path = bag_path
        self.output_path = Path(output_path)
        self.bridge = CvBridge()
        
        # D455 ROS1 topic names (bridged from ROS2)
        self.left_image_topic = "/camera/camera/infra1/image_rect_raw"
        self.right_image_topic = "/camera/camera/infra2/image_rect_raw"
        self.imu_topic = "/camera/camera/imu"
        
        # Output directories
        self.mav0_path = self.output_path / "mav0"
        self.cam0_path = self.mav0_path / "cam0" / "data"
        self.cam1_path = self.mav0_path / "cam1" / "data"
        self.imu0_path = self.mav0_path / "imu0"
        
        print(f"🎒 Noetic Bag to EuRoC Converter")
        print(f"📁 Input bag: {bag_path}")
        print(f"📂 Output dataset: {output_path}")

    def create_directory_structure(self):
        """Create EuRoC dataset directory structure"""
        print("📁 Creating EuRoC directory structure...")
        
        # Create all necessary directories
        self.cam0_path.mkdir(parents=True, exist_ok=True)
        self.cam1_path.mkdir(parents=True, exist_ok=True)
        self.imu0_path.mkdir(parents=True, exist_ok=True)
        
        print(f"✅ Created: {self.mav0_path}")

    def create_yaml_configs(self):
        """Create necessary YAML configuration files"""
        print("📝 Creating YAML configuration files...")
        
        # Create body.yaml (coordinate frame definitions)
        body_config = {
            'T_BS': {
                'data': [1.0, 0.0, 0.0, 0.0,
                        0.0, 1.0, 0.0, 0.0, 
                        0.0, 0.0, 1.0, 0.0,
                        0.0, 0.0, 0.0, 1.0],
                'rows': 4,
                'cols': 4
            }
        }
        
        body_yaml_path = self.mav0_path / "body.yaml"
        with open(body_yaml_path, 'w') as f:
            yaml.dump(body_config, f, default_flow_style=False)
        print(f"✅ Created: {body_yaml_path}")
        
        # Create imu0/sensor.yaml (IMU configuration)
        imu_config = {
            'sensor_type': 'imu',
            'comment': 'Intel RealSense D455 IMU',
            'T_BS': {
                'data': [1.0, 0.0, 0.0, 0.0,
                        0.0, 1.0, 0.0, 0.0,
                        0.0, 0.0, 1.0, 0.0, 
                        0.0, 0.0, 0.0, 1.0],
                'rows': 4,
                'cols': 4
            },
            'rate_hz': 400,
            'gyroscope_noise_density': 0.0003,
            'gyroscope_random_walk': 0.000003,
            'accelerometer_noise_density': 0.008,
            'accelerometer_random_walk': 0.0002
        }
        
        imu_sensor_yaml_path = self.imu0_path / "sensor.yaml"
        with open(imu_sensor_yaml_path, 'w') as f:
            yaml.dump(imu_config, f, default_flow_style=False)
        print(f"✅ Created: {imu_sensor_yaml_path}")

    def ros_time_to_ns(self, ros_time):
        """Convert ROS time to nanoseconds"""
        return int(ros_time.secs * 1e9 + ros_time.nsecs)

    def convert_bag(self):
        """Main conversion function"""
        print("🔄 Opening bag file and extracting data...")
        
        # Data storage
        image_data = []  # [(timestamp_ns, left_msg, right_msg)]
        imu_data = []    # [(timestamp_ns, imu_msg)]
        
        # Statistics
        left_count = 0
        right_count = 0
        imu_count = 0
        stereo_pairs = 0
        
        try:
            bag = rosbag.Bag(self.bag_path, 'r')
            
            # Get bag info
            bag_info = bag.get_type_and_topic_info()
            topics = bag_info[1].keys()
            print(f"📋 Available topics: {list(topics)}")
            
            # Check if required topics exist
            required_topics = [self.left_image_topic, self.right_image_topic, self.imu_topic]
            missing_topics = [t for t in required_topics if t not in topics]
            
            if missing_topics:
                print(f"⚠️  Warning: Missing topics: {missing_topics}")
                print("Available image topics:")
                for topic in topics:
                    if 'image' in topic:
                        print(f"  - {topic}")
                print("Available IMU topics:")
                for topic in topics:
                    if 'imu' in topic or 'gyro' in topic or 'accel' in topic:
                        print(f"  - {topic}")
            
            print("📖 Reading messages from bag...")
            
            # Collect stereo image pairs with timestamps
            left_images = {}  # timestamp_ns -> ImageMsg
            right_images = {} # timestamp_ns -> ImageMsg
            
            # Read all messages
            for topic, msg, t in bag.read_messages():
                timestamp_ns = self.ros_time_to_ns(t)
                
                if topic == self.left_image_topic:
                    left_images[timestamp_ns] = msg
                    left_count += 1
                    
                elif topic == self.right_image_topic:
                    right_images[timestamp_ns] = msg 
                    right_count += 1
                    
                elif topic == self.imu_topic:
                    imu_data.append((timestamp_ns, msg))
                    imu_count += 1
            
            bag.close()
            
            print(f"📊 Extracted {left_count} left images, {right_count} right images, {imu_count} IMU samples")
            
            # Create synchronized stereo pairs
            print("🔄 Synchronizing stereo pairs...")
            for timestamp_ns in sorted(left_images.keys()):
                if timestamp_ns in right_images:
                    image_data.append((timestamp_ns, left_images[timestamp_ns], right_images[timestamp_ns]))
                    stereo_pairs += 1
            
            print(f"✅ Created {stereo_pairs} synchronized stereo pairs")
            
            if stereo_pairs == 0:
                print("❌ No synchronized stereo pairs found! Check your bag topics.")
                return False
                
        except Exception as e:
            print(f"❌ Error reading bag: {e}")
            return False
        
        # Convert and save images
        print("🖼️  Converting and saving images...")
        self.save_images(image_data)
        
        # Convert and save IMU data  
        if imu_data and self.imu_topic in topics:
            print("📊 Converting and saving IMU data...")
            self.save_imu_data(imu_data)
        else:
            print("⚠️  No IMU data found, skipping IMU conversion")
        
        print("✅ Conversion completed successfully!")
        return True

    def save_images(self, image_data):
        """Save stereo images in EuRoC format"""
        for i, (timestamp_ns, left_msg, right_msg) in enumerate(image_data):
            try:
                # Convert ROS images to OpenCV
                left_cv = self.bridge.imgmsg_to_cv2(left_msg, desired_encoding="mono8")
                right_cv = self.bridge.imgmsg_to_cv2(right_msg, desired_encoding="mono8")
                
                # Create filenames with nanosecond timestamps
                filename = f"{timestamp_ns}.png"
                
                # Save images
                left_path = self.cam0_path / filename
                right_path = self.cam1_path / filename
                
                cv2.imwrite(str(left_path), left_cv)
                cv2.imwrite(str(right_path), right_cv)
                
                # Progress update
                if (i + 1) % 100 == 0 or i == len(image_data) - 1:
                    print(f"  💾 Saved {i + 1}/{len(image_data)} stereo pairs")
                    
            except Exception as e:
                print(f"❌ Error processing image pair {i}: {e}")
                continue
        
        print(f"✅ Saved {len(image_data)} stereo pairs to cam0/ and cam1/")

    def save_imu_data(self, imu_data):
        """Save IMU data in EuRoC CSV format"""
        csv_path = self.imu0_path / "data.csv"
        
        # EuRoC IMU CSV header
        header = [
            "#timestamp [ns]",
            "w_RS_S_x [rad s^-1]", "w_RS_S_y [rad s^-1]", "w_RS_S_z [rad s^-1]",
            "a_RS_S_x [m s^-2]", "a_RS_S_y [m s^-2]", "a_RS_S_z [m s^-2]"
        ]
        
        try:
            with open(csv_path, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(header)
                
                for i, (timestamp_ns, imu_msg) in enumerate(imu_data):
                    # Extract IMU data
                    gyr_x = imu_msg.angular_velocity.x
                    gyr_y = imu_msg.angular_velocity.y  
                    gyr_z = imu_msg.angular_velocity.z
                    
                    acc_x = imu_msg.linear_acceleration.x
                    acc_y = imu_msg.linear_acceleration.y
                    acc_z = imu_msg.linear_acceleration.z
                    
                    # Write row
                    row = [timestamp_ns, gyr_x, gyr_y, gyr_z, acc_x, acc_y, acc_z]
                    writer.writerow(row)
                    
                    # Progress update
                    if (i + 1) % 1000 == 0 or i == len(imu_data) - 1:
                        print(f"  📊 Saved {i + 1}/{len(imu_data)} IMU samples")
            
            print(f"✅ Saved {len(imu_data)} IMU samples to {csv_path}")
            
        except Exception as e:
            print(f"❌ Error saving IMU data: {e}")

    def print_summary(self):
        """Print conversion summary"""
        print("\n" + "="*50)
        print("📋 CONVERSION SUMMARY")
        print("="*50)
        print(f"📂 Output dataset: {self.output_path}")
        print(f"📁 Structure:")
        print(f"  mav0/cam0/data/     - Left stereo images") 
        print(f"  mav0/cam1/data/     - Right stereo images")
        print(f"  mav0/imu0/data.csv  - IMU data (EuRoC format)")
        print(f"  mav0/body.yaml      - Coordinate frames")
        print("\n🚀 Ready to run AirSLAM with:")
        print(f"   roslaunch air_slam vo_d455_dataset.launch")
        print("="*50)

def main():
    parser = argparse.ArgumentParser(description='Convert ROS Noetic bag to EuRoC format for D455')
    parser.add_argument('bag_path', help='Path to input ROS bag file')
    parser.add_argument('output_path', help='Path to output EuRoC dataset directory')
    parser.add_argument('--topics', action='store_true', help='List available topics and exit')
    
    args = parser.parse_args()
    
    # Check if bag file exists
    if not os.path.exists(args.bag_path):
        print(f"❌ Error: Bag file '{args.bag_path}' not found!")
        return 1
    
    # List topics mode
    if args.topics:
        print(f"📋 Topics in {args.bag_path}:")
        try:
            bag = rosbag.Bag(args.bag_path, 'r')
            bag_info = bag.get_type_and_topic_info()
            for topic, info in bag_info[1].items():
                print(f"  {topic} ({info.message_count} messages, {info.msg_type})")
            bag.close()
        except Exception as e:
            print(f"❌ Error reading bag: {e}")
        return 0
    
    # Create converter and run
    converter = NoeticBagToEuRoC(args.bag_path, args.output_path)
    
    # Run conversion pipeline  
    converter.create_directory_structure()
    converter.create_yaml_configs()
    
    if converter.convert_bag():
        converter.print_summary()
        return 0
    else:
        print("❌ Conversion failed!")
        return 1

if __name__ == "__main__":
    exit(main())


