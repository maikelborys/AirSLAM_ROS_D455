cmake_minimum_required(VERSION 3.14)
project(air_slam_xfeat LANGUAGES CXX CUDA)

# ============================================================================
# AirSLAM ROS 2 Jazzy port — work in progress
# Phase 0 stub: compiles only the 3rdparty subdirs so colcon graph resolves.
# AirSLAM sources will be added back library-by-library as Phases 1/2 land.
# Original ROS 1 CMakeLists is preserved at CMakeLists.txt.ros1.bak.
# ============================================================================

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# Suppress warning storm from upstream until the port is green.
add_compile_options(-w)

# ----------------------------------------------------------------------------
# ROS 2 / ament dependencies
# ----------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ----------------------------------------------------------------------------
# System dependencies
# ----------------------------------------------------------------------------
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(CUDAToolkit REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization)
find_package(G2O REQUIRED)
find_package(Gflags REQUIRED)
find_package(Glog REQUIRED)

# ----------------------------------------------------------------------------
# 3rdparty (kept as-is from upstream — both have no ROS coupling)
# ----------------------------------------------------------------------------
add_subdirectory(${PROJECT_SOURCE_DIR}/3rdparty/tensorrtbuffer)
add_subdirectory(${PROJECT_SOURCE_DIR}/3rdparty/DBoW2)

# ----------------------------------------------------------------------------
# air_slam_core_lib — non-ROS sources, validates the TRT 10 migration in
# isolation. Sources land here phase-by-phase as Phase 1 progresses.
# ----------------------------------------------------------------------------
add_library(air_slam_core_lib SHARED
  # Networks (TRT 10)
  src/super_point.cpp
  src/super_glue.cpp
  src/light_glue.cpp
  src/plnet.cpp
  src/xfeat.cpp
  src/feature_detector.cc
  src/point_matcher.cc
  # Geometry / IO / utilities
  src/camera.cc
  src/dataset.cc
  src/imu.cc
  src/frame.cc
  src/mappoint.cc
  src/mapline.cc
  src/line_processor.cc
  src/utils.cc
  src/timer.cc
  src/debug.cc
  # BoW
  src/bow/FSuperpoint.cc
  src/bow/database.cc
  # g2o vertices/edges (ROS-free)
  src/g2o_optimization/vertex_line3d.cc
  src/g2o_optimization/vertex_vi_pose.cc
  src/g2o_optimization/vertex_imu.cc
  src/g2o_optimization/edge_project_point.cc
  src/g2o_optimization/edge_project_line.cc
  src/g2o_optimization/edge_imu.cc
  src/g2o_optimization/edge_relative_pose.cc
)

target_include_directories(air_slam_core_lib
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
    ${EIGEN3_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${G2O_INCLUDE_DIR}
    ${GFLAGS_INCLUDE_DIRS}
    ${GLOG_INCLUDE_DIRS}
)

target_link_libraries(air_slam_core_lib
  PUBLIC
    tensorrtbuffer
    DBoW2
    ${OpenCV_LIBRARIES}
    yaml-cpp
    Boost::serialization
    nvinfer
    nvonnxparser
    CUDA::cudart
    ${G2O_LIBRARIES}
    ${GFLAGS_LIBRARIES}
    ${GLOG_LIBRARIES}
)

# ----------------------------------------------------------------------------
# air_slam_lib — ROS-aware sources (RosPublisher, eventually MapBuilder etc.)
# ----------------------------------------------------------------------------
add_library(air_slam_lib SHARED
  src/ros_publisher.cc
  src/map.cc
  src/map_builder.cc
  src/map_refiner.cc
  src/map_user.cc
  src/g2o_optimization/g2o_optimization.cc
)

target_include_directories(air_slam_lib
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(air_slam_lib
  PUBLIC
    air_slam_core_lib
)

ament_target_dependencies(air_slam_lib
  PUBLIC
    rclcpp
    cv_bridge
    image_transport
    geometry_msgs
    nav_msgs
    sensor_msgs
    std_msgs
    visualization_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
)

# ----------------------------------------------------------------------------
# Demo executables
# ----------------------------------------------------------------------------
foreach(demo visual_odometry map_refinement relocalization test_feature test_xfeat)
  add_executable(${demo} demo/${demo}.cpp)
  ament_target_dependencies(${demo} rclcpp)
  target_link_libraries(${demo} air_slam_lib)
  install(TARGETS ${demo} DESTINATION lib/${PROJECT_NAME})
endforeach()

install(TARGETS
  air_slam_core_lib
  air_slam_lib
  tensorrtbuffer
  DBoW2
  EXPORT air_slam_xfeat_export
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# ----------------------------------------------------------------------------
# Install only what currently exists
# ----------------------------------------------------------------------------
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  DIRECTORY configs voc rviz launch output
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*.launch" EXCLUDE
)

ament_export_include_directories(include)
ament_package()
