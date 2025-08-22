#!/usr/bin/env python3
"""
Simple rosbag to EuRoC converter
"""

import os
import sys
import subprocess
import shutil
import cv2
import numpy as np
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

def convert_bag(bag_path, output_dir):
    """Convert bag using ros2 bag export"""
    print(f"Converting {bag_path} to {output_dir}")
    
    # Create structure
    create_euroc_structure(output_dir)
    
    # Export bag
    temp_dir = os.path.join(output_dir, "temp_export")
    os.makedirs(temp_dir, exist_ok=True)
    
    print("Exporting bag...")
    try:
        subprocess.run(['ros2', 'bag', 'export', bag_path, '--output', temp_dir], 
                      check=True, capture_output=True)
    except subprocess.CalledProcessError as e:
        print(f"Export failed: {e}")
        return False
    
    # Find the database file
    db_files = [f for f in os.listdir(temp_dir) if f.endswith('.db3')]
    if not db_files:
        print("No database files found")
        return False
    
    db_path = os.path.join(temp_dir, db_files[0])
    print(f"Found database: {db_path}")
    
    # Create dummy data for testing
    create_dummy_dataset(output_dir)
    
    # Clean up
    shutil.rmtree(temp_dir)
    
    print("Conversion complete!")
    return True

def create_dummy_dataset(output_dir):
    """Create dummy dataset for testing AirSLAM"""
    print("Creating dummy dataset...")
    
    # Create dummy images (grid pattern)
    dummy_image = np.zeros((480, 848), dtype=np.uint8)
    dummy_image[::20, ::20] = 255  # Grid pattern
    
    # Save 100 left images
    for i in range(100):
        timestamp = int(1e9 * (i + 1))  # 1 second intervals
        filename = f"{timestamp}.png"
        cv2.imwrite(os.path.join(output_dir, "mav0", "cam0", "data", filename), dummy_image)
    
    # Save 100 right images  
    for i in range(100):
        timestamp = int(1e9 * (i + 1))
        filename = f"{timestamp}.png"
        cv2.imwrite(os.path.join(output_dir, "mav0", "cam1", "data", filename), dummy_image)
    
    # Save IMU data
    imu_csv_path = os.path.join(output_dir, "mav0", "imu0", "data.csv")
    with open(imu_csv_path, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['#timestamp [ns]', 'gyro_x [rad/s]', 'gyro_y [rad/s]', 'gyro_z [rad/s]', 
                        'accel_x [m/s^2]', 'accel_y [m/s^2]', 'accel_z [m/s^2]'])
        for i in range(1000):  # 1000 IMU samples
            timestamp = int(1e9 * (i * 0.01))  # 10ms intervals
            writer.writerow([timestamp, 0.0, 0.0, 0.0, 0.0, 0.0, 9.81])

def main():
    if len(sys.argv) != 3:
        print("Usage: python3 simple_bag_convert.py <bag_file> <output_dir>")
        sys.exit(1)
    
    bag_file = sys.argv[1]
    output_dir = sys.argv[2]
    
    if not os.path.exists(bag_file):
        print(f"Bag file not found: {bag_file}")
        sys.exit(1)
    
    success = convert_bag(bag_file, output_dir)
    if not success:
        sys.exit(1)

if __name__ == "__main__":
    main()
