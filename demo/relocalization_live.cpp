#include <iostream>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <ros/ros.h>
#include <thread>

#include "read_configs.h"
#include "map_user.h"
#include "ros_dataset.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "relocalization_live");
  ros::NodeHandle nh;

  // Get parameters
  std::string config_path, model_dir, map_root, voc_path, traj_path;
  ros::param::get("~config_path", config_path);
  ros::param::get("~model_dir", model_dir);
  ros::param::get("~map_root", map_root);
  ros::param::get("~voc_path", voc_path);
  ros::param::get("~traj_path", traj_path);

  // Check for live topics mode
  bool use_live_topics = false;
  ros::param::get("~use_live_topics", use_live_topics);

  RelocalizationConfigs configs(config_path, model_dir);
  ros::param::get("~camera_config_path", configs.camera_config_path);

  // Initialize map user
  MapUser map_user(configs, nh);
  map_user.LoadMap(map_root);
  map_user.LoadVocabulary(voc_path);
  
  ROS_INFO("=== D455 LIVE RELOCALIZATION ===");
  ROS_INFO("Loaded map from: %s", map_root.c_str());
  ROS_INFO("Using vocabulary: %s", voc_path.c_str());

  std::vector<std::pair<std::string, Eigen::Matrix4d>> trajectory;
  Eigen::Matrix4d base_frame_pose = map_user.GetBaseFramePose();
  double base_frame_time = map_user.GetBaseFrameTimestamp();
  trajectory.emplace_back(std::make_pair(("base "+DoubleTimeToString(base_frame_time)), base_frame_pose));

  if (use_live_topics) {
    ROS_INFO("=== USING LIVE ROS TOPICS FOR LOCALIZATION ===");
    
    // Create ROS dataset for live topics (no IMU needed for localization)
    ROSDataset ros_dataset(nh, false); // IMU disabled for localization
    ROS_INFO("ROS dataset initialized for live localization");
    
    int success_num = 0;
    int total_attempts = 0;
    double sum_time = 0;
    
    ROS_INFO("Waiting for camera data on topics...");
    ROS_INFO("Ready for real-time localization!");
    
    // Live processing loop 
    ros::Rate loop_rate(10); // 10 Hz for localization (slower than VO)
    while (ros::ok()) {
      ros::spinOnce(); // Process ROS callbacks
      
      // Check if new data is available
      if (!ros_dataset.HasNewData()) {
        loop_rate.sleep();
        continue;
      }
      
      cv::Mat image_left, image_right;
      double timestamp;
      ImuDataList batch_imu_data; // Not used for localization
      
      // Get data from ROS topics (only need left image for localization)
      if (!ros_dataset.GetData(0, image_left, image_right, batch_imu_data, timestamp)) {
        loop_rate.sleep();
        continue;
      }
      
      total_attempts++;
      std::cout << "Localization attempt " << total_attempts << ", timestamp: " << std::fixed << timestamp << std::endl;
      
      // Perform relocalization using left image only
      Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
      std::string result_idx = "fail_" + std::to_string(total_attempts);

      auto before_infer = std::chrono::high_resolution_clock::now();   
      bool success = map_user.Relocalization(image_left, pose);
      auto after_infer = std::chrono::high_resolution_clock::now();
      
      auto cost_time = std::chrono::duration_cast<std::chrono::milliseconds>(after_infer - before_infer).count();
      sum_time += (double)cost_time;
      
      if (success) {
        result_idx = "success_" + std::to_string(total_attempts);
        success_num++;
        ROS_INFO("✅ LOCALIZATION SUCCESS! Frame %d (%.1f%% success rate)", 
                 total_attempts, 100.0 * success_num / total_attempts);
        
        // Print pose information
        Eigen::Vector3d translation = pose.block<3,1>(0, 3);
        std::cout << "📍 Position: [" << translation.x() << ", " << translation.y() << ", " << translation.z() << "]" << std::endl;
      } else {
        ROS_WARN("❌ Localization failed for frame %d (%.1f%% success rate)", 
                 total_attempts, 100.0 * success_num / total_attempts);
      }
      
      std::cout << "Relocalization Time: " << cost_time << " ms." << std::endl;
      trajectory.emplace_back(std::make_pair(result_idx, pose));
      
      loop_rate.sleep();
    }
    
    if (total_attempts > 0) {
      std::cout << "\n=== LOCALIZATION SUMMARY ===" << std::endl;
      std::cout << "Total attempts: " << total_attempts << std::endl;
      std::cout << "Successful localizations: " << success_num << std::endl;
      std::cout << "Success rate: " << (100.0 * success_num / total_attempts) << "%" << std::endl;
      std::cout << "Average processing time: " << (sum_time / total_attempts) << " ms" << std::endl;
    }
    
  } else {
    ROS_ERROR("Live topics mode not enabled! Set use_live_topics:=true");
    return -1;
  }

  // Save trajectory
  SaveTumTrajectoryToFile(traj_path, trajectory);
  ROS_INFO("Trajectory saved to: %s", traj_path.c_str());

  map_user.StopVisualization();
  ros::shutdown();

  return 0;
}
