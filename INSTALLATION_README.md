# AirSLAM Installation Guide

This guide provides step-by-step instructions for installing AirSLAM from source on Ubuntu 22.04 with all dependencies built locally.

## System Requirements

- **OS**: Ubuntu 22.04 (Jammy Jellyfish)
- **CUDA**: 12.1 (with compatible NVIDIA driver)
- **TensorRT**: 8.6.1.6
- **ROS**: Noetic (built from source)
- **RAM**: Minimum 8GB (16GB recommended)
- **Storage**: At least 10GB free space

## Prerequisites

### 1. Update System Packages
```bash
sudo apt update && sudo apt upgrade -y
```

### 2. Install Basic Build Tools
```bash
sudo apt install -y build-essential cmake git wget unzip curl
```

### 3. Install System Dependencies
```bash
sudo apt install -y \
    libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev \
    libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev \
    libboost-all-dev libeigen3-dev libsuitesparse-dev libgoogle-glog-dev \
    libgflags-dev libatlas-base-dev libhdf5-dev libgtest-dev \
    libyaml-cpp-dev libgflags-dev libgoogle-glog-dev \
    libgl1-mesa-dev libglu1-mesa-dev freeglut3-dev libqt5opengl5-dev
```

## Dependency Installation

All dependencies will be installed to `/usr/local/` to avoid conflicts with system packages.

### 1. OpenCV 4.7.0 (with CUDA 12.1 support)

```bash
cd ~
wget -O opencv_4.7.zip https://github.com/opencv/opencv/archive/4.7.0.zip
wget -O opencv_contrib_4.7.zip https://github.com/opencv/opencv_contrib/archive/4.7.0.zip

unzip opencv_4.7.zip
unzip opencv_contrib_4.7.zip
mv opencv-4.7.0 opencv_4_7
mv opencv_contrib-4.7.0 opencv_contrib_4_7

cd opencv_4_7
mkdir build && cd build

cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D INSTALL_PYTHON_EXAMPLES=OFF \
    -D INSTALL_C_EXAMPLES=OFF \
    -D OPENCV_ENABLE_NONFREE=ON \
    -D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib_4_7/modules \
    -D PYTHON_EXECUTABLE=$(which python3) \
    -D BUILD_EXAMPLES=OFF \
    -D WITH_CUDA=ON \
    -D WITH_CUDNN=ON \
    -D OPENCV_DNN_CUDA=ON \
    -D ENABLE_FAST_MATH=ON \
    -D CUDA_FAST_MATH=ON \
    -D CUDA_ARCH_BIN=8.6 \
    -D WITH_CUBLAS=ON \
    -D WITH_TBB=ON \
    -D WITH_OPENMP=ON \
    -D WITH_GTK=ON \
    -D WITH_FFMPEG=ON \
    -D WITH_GSTREAMER=OFF \
    -D WITH_1394=OFF \
    -D WITH_OPENEXR=OFF \
    -D WITH_OPENCL=OFF \
    -D WITH_IPP=OFF \
    -D WITH_PROTOBUF=OFF \
    -D WITH_QUIRC=OFF \
    -D WITH_ADE=OFF \
    -D WITH_FREETYPE=ON \
    -D WITH_HARFBUZZ=ON \
    -D WITH_PTHREADS_PF=ON \
    -D WITH_DIRECTX=OFF \
    -D WITH_VA=ON \
    -D WITH_VA_INTEL=ON \
    -D WITH_GDAL=OFF \
    -D WITH_XINE=OFF \
    -D BUILD_PERF_TESTS=OFF \
    -D BUILD_TESTS=OFF ..

make -j$(nproc)
sudo make install
sudo ldconfig
```

### 2. Eigen 3.4.0

```bash
cd ~
wget https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
tar -xzf eigen-3.4.0.tar.gz
mv eigen-3.4.0 eigen_3_4_0

cd eigen_3_4_0
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local
sudo make install
sudo ldconfig
```

### 3. Ceres 2.0.0

```bash
cd ~
wget https://github.com/ceres-solver/ceres-solver/archive/2.0.0.tar.gz
tar -xzf 2.0.0.tar.gz
mv ceres-solver-2.0.0 ceres_2_0_0

cd ceres_2_0_0
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF
make -j$(nproc)
sudo make install
sudo ldconfig
```

### 4. G2O (tag: 20230223_git) with OpenGL Support

```bash
cd ~
git clone https://github.com/RainerKuemmerle/g2o.git g2o_20230223
cd g2o_20230223
git checkout 20230223_git

mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local -DBUILD_SHARED_LIBS=ON -DWITH_OPENGL=ON
make -j$(nproc)
sudo make install
sudo ldconfig
```

### 5. ROS Noetic (Built from Source)

Since ROS Noetic doesn't officially support Ubuntu 22.04, we build it from source:

```bash
# Add ROS repository (for dependencies)
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# Install ROS Noetic from source
mkdir -p ~/ros_catkin_ws/src
cd ~/ros_catkin_ws/src

# Download ROS Noetic source
wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.install
chmod +x ros.install
./ros.install noetic

# Build ROS
cd ~/ros_catkin_ws
./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release

# Add to bashrc
echo "source ~/ros_catkin_ws/install_isolated/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## AirSLAM Installation

### 1. Clone AirSLAM Repository

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/sair-lab/AirSLAM.git
```

### 2. Modify CMakeLists.txt

Edit `~/catkin_ws/src/AirSLAM/CMakeLists.txt` to use OpenCV 4.7:

```cmake
# Change line 27 from:
find_package(OpenCV 4.2 REQUIRED)
# To:
find_package(OpenCV 4.7 REQUIRED)

# Add Ceres dependency (around line 32):
find_package(Ceres REQUIRED)

# Add Ceres include directories (around line 53):
${CERES_INCLUDE_DIRS}

# Add Ceres libraries (around line 99):
${CERES_LIBRARIES}
```

### 3. Build AirSLAM

```bash
cd ~/catkin_ws
source ~/ros_catkin_ws/install_isolated/setup.bash
catkin_make
```

### 4. Source the Workspace

```bash
source ~/catkin_ws/devel/setup.bash
```

## Verification

### Check Installed Libraries

```bash
# Verify OpenCV
/usr/local/bin/opencv_version

# Verify Eigen
pkg-config --modversion eigen3

# Verify G2O libraries
find /usr/local/lib -name "*g2o*" | grep -E "(freeglut|opengl|simulator)"

# Verify AirSLAM executables
ls -la ~/catkin_ws/devel/lib/air_slam/
```

### Test AirSLAM Installation

```bash
# Source environments
source ~/ros_catkin_ws/install_isolated/setup.bash
source ~/catkin_ws/devel/setup.bash

# Test an executable
~/catkin_ws/devel/lib/air_slam/test_feature --help
```

## Usage

### Running AirSLAM

```bash
# Source environments
source ~/ros_catkin_ws/install_isolated/setup.bash
source ~/catkin_ws/devel/setup.bash

# Run visual odometry (example with EuRoC dataset)
roslaunch air_slam vo_euroc.launch

# Run map refinement
roslaunch air_slam mr_euroc.launch

# Run relocalization
roslaunch air_slam reloc_euroc.launch
```

### Configuration

Edit the launch files in `~/catkin_ws/src/AirSLAM/launch/` to configure:
- Data paths (`dataroot`)
- Map saving directory (`saving_dir`)
- Camera configuration files

Available camera configs:
- `configs/camera/euroc.yaml` - EuRoC dataset
- `configs/camera/tartanair.yaml` - TartanAir dataset
- `configs/camera/uma_bumblebee.yaml` - UMA Bumblebee
- `configs/camera/dark_euroc.yaml` - Dark EuRoC dataset
- `configs/camera/oivio.yaml` - OIVIO dataset

## Troubleshooting

### Common Issues and Solutions

#### 1. OpenCV 4.2 CUDA Compatibility Error
**Error**: OpenCV 4.2 build fails with CUDA 12.1 due to deprecated texture APIs:
```
error: 'textureReference' is deprecated
error: 'cudaUnbindTexture' is deprecated
```

**Solution**: Upgrade to OpenCV 4.7.0 which is compatible with CUDA 12.1. The installation guide above uses OpenCV 4.7.0 instead of 4.2.

#### 2. Ceres TBB Compatibility Error
**Error**: Ceres 2.0.0 CMake configuration fails with:
```
CMake Error: Could not find tbb_stddef.h
```

**Solution**: Ubuntu 22.04 uses `oneTBB` instead of legacy TBB. Create a compatibility header:
```bash
sudo mkdir -p /usr/include/tbb
sudo tee /usr/include/tbb/tbb_stddef.h << 'EOF'
/* Compatibility shim for oneTBB (Ubuntu 22.04) to satisfy legacy FindTBB.cmake */
#pragma once
#include <oneapi/tbb/version.h>
#ifndef TBB_COMPATIBLE_INTERFACE_VERSION
#define TBB_COMPATIBLE_INTERFACE_VERSION TBB_INTERFACE_VERSION
#endif
EOF
```

#### 3. G2O Missing Libraries Error
**Error**: AirSLAM build fails with missing G2O libraries:
```
G2O_EXT_FREEGLUT_MINIMAL_LIBRARY-NOTFOUND
G2O_OPENGL_HELPER_LIBRARY-NOTFOUND
G2O_SIMULATOR_LIBRARY-NOTFOUND
```

**Solution**: Install OpenGL development packages and rebuild G2O with OpenGL support:
```bash
sudo apt install -y libglu1-mesa-dev libgl1-mesa-dev libqt5opengl5-dev freeglut3-dev
cd ~/g2o_20230223
rm -rf build
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local -DBUILD_SHARED_LIBS=ON -DWITH_OPENGL=ON
make -j$(nproc)
sudo make install
sudo ldconfig
```

#### 4. ROS Noetic Not Found Error
**Error**: 
```bash
source /opt/ros/noetic/setup.bash
# bash: /opt/ros/noetic/setup.bash: No such file or directory
```

**Solution**: ROS Noetic doesn't officially support Ubuntu 22.04, so we build it from source as shown in the installation guide above.

#### 5. System Package Installation Error
**Error**: Some packages not found during system dependency installation:
```
E: Package 'libdc1394-22-dev' has no installation candidate
```

**Solution**: Skip problematic packages and install the rest:
```bash
sudo apt install -y \
    libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev \
    libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev \
    libboost-all-dev libeigen3-dev libsuitesparse-dev libgoogle-glog-dev \
    libgflags-dev libatlas-base-dev libhdf5-dev libgtest-dev \
    libyaml-cpp-dev libgflags-dev libgoogle-glog-dev \
    libgl1-mesa-dev libglu1-mesa-dev freeglut3-dev libqt5opengl5-dev
```

### Build Errors

- **OpenCV CUDA errors**: Ensure CUDA 12.1 is properly installed and compatible with your NVIDIA driver
- **G2O missing libraries**: Install OpenGL development packages and rebuild G2O with explicit OpenGL support
- **Ceres TBB issues**: The compatibility header resolves this automatically
- **Memory Issues**: Reduce `-j` flag in make commands if you encounter memory problems (e.g., use `make -j4` instead of `make -j$(nproc)`)

### Verification Commands

After installation, verify everything works:
```bash
# Check OpenCV
/usr/local/bin/opencv_version

# Check G2O libraries
find /usr/local/lib -name "*g2o*" | grep -E "(freeglut|opengl|simulator)"

# Check AirSLAM executables
ls -la ~/catkin_ws/devel/lib/air_slam/

# Test an executable
~/catkin_ws/devel/lib/air_slam/test_feature --help
```

## File Structure

```
~/catkin_ws/
├── src/
│   └── AirSLAM/           # AirSLAM source code
├── devel/
│   └── lib/
│       └── air_slam/      # Built executables
└── build/                 # Build files

/usr/local/
├── include/               # Header files
├── lib/                   # Shared libraries
└── bin/                   # Executables
```

## Dependencies Summary

| Dependency | Version | Location | Purpose |
|------------|---------|----------|---------|
| OpenCV | 4.7.0 | `/usr/local/` | Computer vision library |
| Eigen | 3.4.0 | `/usr/local/` | Linear algebra library |
| Ceres | 2.0.0 | `/usr/local/` | Optimization library |
| G2O | 20230223_git | `/usr/local/` | Graph optimization |
| ROS | Noetic | `~/ros_catkin_ws/` | Robot operating system |
| CUDA | 12.1 | System | GPU acceleration |
| TensorRT | 8.6.1.6 | System | Deep learning inference |

## Support

For issues related to:
- **AirSLAM**: Check the [original repository](https://github.com/sair-lab/AirSLAM)
- **Dependencies**: Refer to individual project documentation
- **Installation**: This guide covers Ubuntu 22.04 specifically

---

**Note**: This installation guide is specifically tailored for Ubuntu 22.04 with CUDA 12.1. For other systems, some modifications may be required.
