#!/bin/bash
"""
Convert D455 ROS2 bag to EuRoC format
Usage: ./convert_d455_bag.sh <bag_path> <output_dir>
"""

if [ $# -ne 2 ]; then
    echo "Usage: $0 <bag_path> <output_dir>"
    echo "Example: $0 /home/robot/datasets/bag_d455_1/bag_d455_1_0.db3 /home/robot/datasets/d455_euroc_dataset"
    exit 1
fi

BAG_PATH=$1
OUTPUT_DIR=$2

echo "Converting $BAG_PATH to EuRoC format in $OUTPUT_DIR"

# Check if bag exists
if [ ! -f "$BAG_PATH" ]; then
    echo "Error: Bag file $BAG_PATH not found"
    exit 1
fi

# Source ROS2
source /opt/ros/humble/setup.bash

# Create output directory
mkdir -p "$OUTPUT_DIR"

# Start the converter node in the background
echo "Starting converter node..."
python3 /home/robot/catkin_ws/src/AirSLAM/scripts/bag_to_euroc_subscriber.py "$OUTPUT_DIR" &
CONVERTER_PID=$!

# Wait a bit for the converter to start
sleep 2

# Play the bag
echo "Playing bag file..."
ros2 bag play "$BAG_PATH" --rate 2.0

# Wait a bit more for final messages to be processed
echo "Waiting for final messages to be processed..."
sleep 3

# Kill the converter
echo "Stopping converter..."
kill $CONVERTER_PID 2>/dev/null

# Wait for process to stop
sleep 2

echo "Conversion complete!"
echo "Dataset saved in: $OUTPUT_DIR"

# Show final statistics
echo ""
echo "Dataset contents:"
LEFT_IMAGES=$(find "$OUTPUT_DIR/mav0/cam0/data" -name "*.png" 2>/dev/null | wc -l)
RIGHT_IMAGES=$(find "$OUTPUT_DIR/mav0/cam1/data" -name "*.png" 2>/dev/null | wc -l)
IMU_LINES=$(wc -l < "$OUTPUT_DIR/mav0/imu0/data.csv" 2>/dev/null || echo "0")

echo "Left images: $LEFT_IMAGES"
echo "Right images: $RIGHT_IMAGES" 
echo "IMU samples: $((IMU_LINES - 1))"  # Subtract header

echo ""
echo "You can now run AirSLAM with:"
echo "roslaunch air_slam vo_d455_dataset.launch"


