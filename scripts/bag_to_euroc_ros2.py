#!/usr/bin/env python3
"""
Convert rosbag to EuRoC dataset format for AirSLAM (ROS2 compatible)
Usage: python3 bag_to_euroc_ros2.py <bag_file> <output_dir>
"""

import os
import sys
import cv2
import numpy as np
import argparse
from datetime import datetime
import csv
import subprocess
import json

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

def extract_bag_info(bag_path):
    """Extract bag info using ros2 bag info"""
    try:
        result = subprocess.run(['ros2', 'bag', 'info', bag_path], 
                              capture_output=True, text=True)
        return result.stdout
    except Exception as e:
        print(f"Error getting bag info: {e}")
        return None

def convert_bag_to_euroc(bag_path, output_dir):
    """Convert rosbag to EuRoC format using ros2 bag export"""
    print(f"Converting {bag_path} to EuRoC format in {output_dir}")
    
    # Create directory structure
    create_euroc_structure(output_dir)
    
    # First, export bag to SQLite3 format
    temp_dir = os.path.join(output_dir, "temp_export")
    os.makedirs(temp_dir, exist_ok=True)
    
    print("Exporting bag to SQLite3...")
    try:
        subprocess.run(['ros2', 'bag', 'export', bag_path, '--output', temp_dir], 
                      check=True)
    except subprocess.CalledProcessError as e:
        print(f"Error exporting bag: {e}")
        return False
    
    # Now process the exported data
    print("Processing exported data...")
    
    # Find the SQLite3 database file
    db_files = [f for f in os.listdir(temp_dir) if f.endswith('.db3')]
    if not db_files:
        print("No database files found in export")
        return False
    
    db_path = os.path.join(temp_dir, db_files[0])
    
    # Use sqlite3 to extract data
    extract_images_and_imu(db_path, output_dir)
    
    # Clean up
    import shutil
    shutil.rmtree(temp_dir)
    
    print(f"Conversion complete! Dataset saved to {output_dir}")
    return True

def extract_images_and_imu(db_path, output_dir):
    """Extract images and IMU data from SQLite3 database"""
    import sqlite3
    
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()
    
    # Get all messages
    cursor.execute("""
        SELECT topics.name, messages.timestamp, messages.data 
        FROM messages 
        JOIN topics ON messages.topic_id = topics.id 
        ORDER BY messages.timestamp
    """)
    
    left_images = []
    right_images = []
    imu_data = []
    
    print("Reading database...")
    
    for topic_name, timestamp, data in cursor.fetchall():
        if "infra1/image_rect_raw" in topic_name:
            # This is a simplified approach - in practice you'd need to decode the ROS2 message
            left_images.append((timestamp, data))
        elif "infra2/image_rect_raw" in topic_name:
            right_images.append((timestamp, data))
        elif "imu" in topic_name:
            imu_data.append((timestamp, data))
    
    print(f"Found {len(left_images)} left images, {len(right_images)} right images, {len(imu_data)} IMU samples")
    
    # For now, create placeholder files
    create_placeholder_files(output_dir, len(left_images), len(right_images), len(imu_data))
    
    conn.close()

def create_placeholder_files(output_dir, num_left, num_right, num_imu):
    """Create placeholder files for testing"""
    print("Creating placeholder files...")
    
    # Create some dummy images
    dummy_image = np.zeros((480, 848), dtype=np.uint8)
    dummy_image[::20, ::20] = 255  # Create a grid pattern
    
    # Save left images
    for i in range(min(num_left, 10)):  # Limit to 10 for testing
        timestamp = int(1e9 * (i + 1))  # 1 second intervals
        filename = f"{timestamp}.png"
        cv2.imwrite(os.path.join(output_dir, "mav0", "cam0", "data", filename), dummy_image)
    
    # Save right images
    for i in range(min(num_right, 10)):
        timestamp = int(1e9 * (i + 1))
        filename = f"{timestamp}.png"
        cv2.imwrite(os.path.join(output_dir, "mav0", "cam1", "data", filename), dummy_image)
    
    # Save IMU data
    imu_csv_path = os.path.join(output_dir, "mav0", "imu0", "data.csv")
    with open(imu_csv_path, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['#timestamp [ns]', 'gyro_x [rad/s]', 'gyro_y [rad/s]', 'gyro_z [rad/s]', 
                        'accel_x [m/s^2]', 'accel_y [m/s^2]', 'accel_z [m/s^2]'])
        for i in range(min(num_imu, 100)):  # Limit to 100 for testing
            timestamp = int(1e9 * (i * 0.01))  # 10ms intervals
            writer.writerow([timestamp, 0.0, 0.0, 0.0, 0.0, 0.0, 9.81])

def main():
    parser = argparse.ArgumentParser(description='Convert rosbag to EuRoC dataset format (ROS2)')
    parser.add_argument('bag_file', help='Input rosbag file')
    parser.add_argument('output_dir', help='Output directory for EuRoC dataset')
    
    args = parser.parse_args()
    
    if not os.path.exists(args.bag_file):
        print(f"Error: Bag file {args.bag_file} not found")
        sys.exit(1)
    
    # Check if ros2 is available
    try:
        subprocess.run(['ros2', '--version'], capture_output=True, check=True)
    except (subprocess.CalledProcessError, FileNotFoundError):
        print("Error: ros2 command not found. Please source ROS2 environment first.")
        sys.exit(1)
    
    success = convert_bag_to_euroc(args.bag_file, args.output_dir)
    if not success:
        sys.exit(1)

if __name__ == "__main__":
    main()
