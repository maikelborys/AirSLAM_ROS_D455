#!/bin/bash
# 🎒 ROS Noetic Bag to EuRoC Converter Script
# Converts D455 bag files recorded in ROS1/Noetic to EuRoC format for AirSLAM

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Help function
show_help() {
    echo "🎒 ROS Noetic Bag to EuRoC Converter for AirSLAM"
    echo ""
    echo "Usage: $0 <bag_file> [output_directory]"
    echo ""
    echo "Arguments:"
    echo "  bag_file         Path to ROS1/Noetic bag file"
    echo "  output_directory Optional output directory (default: ~/datasets/)"
    echo ""
    echo "Examples:"
    echo "  $0 my_d455_session.bag                    # Output to ~/datasets/my_d455_session_euroc/"
    echo "  $0 data/slam.bag /tmp/datasets/          # Output to /tmp/datasets/slam_euroc/"
    echo ""
    echo "Options:"
    echo "  --topics         List topics in bag file and exit"
    echo "  --help           Show this help message"
    echo ""
    echo "Required ROS1 topics in bag:"
    echo "  /camera/camera/infra1/image_rect_raw     - Left stereo image"
    echo "  /camera/camera/infra2/image_rect_raw     - Right stereo image"  
    echo "  /camera/camera/imu                       - IMU data (optional)"
}

# Check arguments
if [ $# -lt 1 ]; then
    print_error "Missing bag file argument!"
    echo ""
    show_help
    exit 1
fi

# Handle help and options
case $1 in
    --help|-h)
        show_help
        exit 0
        ;;
    --topics)
        if [ $# -lt 2 ]; then
            print_error "Missing bag file for --topics option"
            exit 1
        fi
        print_status "Listing topics in $2..."
        source /home/robot/ros_catkin_ws/install_isolated/setup.bash
        python3 $(dirname "$0")/scripts/noetic_bag_to_euroc.py "$2" /tmp --topics
        exit 0
        ;;
esac

# Parse arguments
BAG_FILE="$1"
OUTPUT_BASE="${2:-$HOME/datasets}"

# Validate bag file
if [ ! -f "$BAG_FILE" ]; then
    print_error "Bag file '$BAG_FILE' not found!"
    exit 1
fi

# Create output directory name
BAG_NAME=$(basename "$BAG_FILE" .bag)
OUTPUT_DIR="$OUTPUT_BASE/${BAG_NAME}_euroc"

print_status "🎒 Starting ROS Noetic Bag Conversion"
echo "📁 Input bag: $BAG_FILE"
echo "📂 Output dataset: $OUTPUT_DIR"
echo ""

# Source ROS environment
print_status "🔧 Setting up ROS environment..."
if [ -f "/home/robot/ros_catkin_ws/install_isolated/setup.bash" ]; then
    source /home/robot/ros_catkin_ws/install_isolated/setup.bash
    print_success "ROS1 Noetic environment activated"
elif [ -f "/opt/ros/noetic/setup.bash" ]; then
    source /opt/ros/noetic/setup.bash
    print_success "ROS1 Noetic (system) environment activated"
else
    print_error "ROS1 Noetic not found! Please install or source manually."
    exit 1
fi

# Check Python dependencies
print_status "📦 Checking Python dependencies..."
python3 -c "import rosbag, cv2, yaml; print('✅ All dependencies available')" || {
    print_error "Missing Python dependencies! Install with:"
    echo "  sudo apt install python3-rosbag python3-opencv python3-yaml python3-cv-bridge"
    exit 1
}

# Create output directory if it doesn't exist
mkdir -p "$(dirname "$OUTPUT_DIR")"

# Run conversion
print_status "🔄 Starting conversion..."
echo ""

CONVERTER_SCRIPT="$(dirname "$0")/scripts/noetic_bag_to_euroc.py"

if [ ! -f "$CONVERTER_SCRIPT" ]; then
    print_error "Converter script not found at $CONVERTER_SCRIPT"
    exit 1
fi

# Execute conversion with error handling
if python3 "$CONVERTER_SCRIPT" "$BAG_FILE" "$OUTPUT_DIR"; then
    echo ""
    print_success "🎉 Conversion completed successfully!"
    echo ""
    echo "📂 Dataset created at: $OUTPUT_DIR"
    echo ""
    echo "🚀 Ready to run AirSLAM:"
    echo "   rosenv"
    echo "   # Update launch file to point to: $OUTPUT_DIR/mav0"  
    echo "   roslaunch air_slam vo_d455_dataset.launch"
    echo ""
    echo "📊 Dataset structure:"
    echo "   $(ls -la "$OUTPUT_DIR/mav0" 2>/dev/null | wc -l) directories created"
    if [ -d "$OUTPUT_DIR/mav0/cam0/data" ]; then
        echo "   $(ls "$OUTPUT_DIR/mav0/cam0/data" | wc -l) stereo image pairs"
    fi
    if [ -f "$OUTPUT_DIR/mav0/imu0/data.csv" ]; then
        echo "   $(wc -l < "$OUTPUT_DIR/mav0/imu0/data.csv") IMU samples"
    fi
else
    echo ""
    print_error "❌ Conversion failed!"
    print_warning "Check the error messages above for details."
    print_warning "Common issues:"
    echo "   - Wrong topic names (use --topics to list)"
    echo "   - Corrupted bag file" 
    echo "   - Insufficient disk space"
    echo "   - Missing ROS dependencies"
    exit 1
fi


