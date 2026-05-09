// AirSLAM Jazzy port: ROS 2 / rclcpp version of the upstream map_refinement
// demo. Loads a map saved by visual_odometry, runs offline pose-graph + global
// bundle adjustment, saves a refined map.
#include <iostream>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>

#include "utils.h"
#include "read_configs.h"
#include "map.h"
#include "map_refiner.h"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("air_slam_map_refinement");

  int breakpoint = 0;
  std::string config_path, model_dir, map_root, voc_path;
  node->declare_parameter<int>("breakpoint", 0);
  node->declare_parameter<std::string>("config_path", "");
  node->declare_parameter<std::string>("model_dir", "");
  node->declare_parameter<std::string>("map_root", "");
  node->declare_parameter<std::string>("voc_path", "");
  node->get_parameter("breakpoint", breakpoint);
  node->get_parameter("config_path", config_path);
  node->get_parameter("model_dir", model_dir);
  node->get_parameter("map_root", map_root);
  node->get_parameter("voc_path", voc_path);

  MapRefinementConfigs configs(config_path, model_dir);
  MapRefiner map_refiner(configs, node);

  std::cout << "Loading map and vocabulary..." << std::endl;
  map_refiner.LoadMap(map_root);
  map_refiner.LoadVocabulary(voc_path);
  std::cout << "Done." << std::endl;

  map_refiner.Wait(breakpoint);

  std::cout << "Building covisibility graph..." << std::endl;
  map_refiner.UpdateCovisibilityGraph();
  std::cout << "Done." << std::endl;

  std::cout << "Loop detection..." << std::endl;
  int loop_num = map_refiner.LoopDetection();
  std::cout << "Done, " << loop_num << " loop pairs are found." << std::endl;

  std::cout << "Optimizing pose graph..." << std::endl;
  map_refiner.PoseGraphRefinement();
  std::cout << "Done." << std::endl;

  map_refiner.Wait(breakpoint);

  std::cout << "Merging mappoints..." << std::endl;
  map_refiner.MergeMap();
  std::cout << "Done." << std::endl;

  map_refiner.Wait(breakpoint);

  std::cout << "Optimizing global map..." << std::endl;
  map_refiner.GlobalMapOptimization();
  map_refiner.UpdateCovisibilityGraph();
  std::cout << "Done." << std::endl;

  std::cout << "Build junction database..." << std::endl;
  map_refiner.BuildJunctionDatabase();
  std::cout << "Done." << std::endl;

  std::string trajectory_global_ba_path = ConcatenateFolderAndFileName(map_root, "trajectory_v1.txt");
  map_refiner.SaveTrajectory(trajectory_global_ba_path);

  map_refiner.Wait(breakpoint);

  std::cout << "Saving final map..." << std::endl;
  map_refiner.SaveFinalMap(map_root);
  std::cout << "Done." << std::endl;

  // Upstream calls exit(0) here because the visualisation thread holds the
  // process. Preserve that behaviour; rclcpp::shutdown() below is for the
  // alternative graceful path if exit is ever removed.
  map_refiner.StopVisualization();
  rclcpp::shutdown();
  return 0;
}
