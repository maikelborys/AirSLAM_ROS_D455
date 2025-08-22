#include <iostream>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <ros/ros.h>
#include <thread>

#include "read_configs.h"
#include "dataset.h"           // Original file-based dataset
#include "ros_dataset.h"       // New ROS topic-based dataset
#include "map_builder.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "air_slam_live");

  std::string config_path, model_dir;
  ros::param::get("~config_path", config_path);
  ros::param::get("~model_dir", model_dir);
  VisualOdometryConfigs configs(config_path, model_dir);
  std::cout << "config done" << std::endl;

  ros::param::get("~dataroot", configs.dataroot);
  ros::param::get("~camera_config_path", configs.camera_config_path);
  ros::param::get("~saving_dir", configs.saving_dir);

  // NEW: Parameter to choose between file-based and topic-based input
  bool use_live_topics = false;
  ros::param::get("~use_live_topics", use_live_topics);
  
  ros::NodeHandle nh;
  MapBuilder map_builder(configs, nh);
  std::cout << "map_builder done" << std::endl;

  if (use_live_topics) {
    ROS_INFO("=== USING LIVE ROS TOPICS ===");
    
    // NEW: Create ROS dataset for live topics
    ROSDataset ros_dataset(nh, map_builder.UseIMU());
    std::cout << "ros_dataset done" << std::endl;
    
    double sum_time = 0;
    int image_num = 0;
    
    ROS_INFO("Waiting for camera data on topics...");
    
    // SIMPLE: Topic-based processing loop 
    ros::Rate loop_rate(30); // Simple 30 Hz rate
    while (ros::ok()) {
      ros::spinOnce(); // Process ROS callbacks
      
      // Check if new data is available
      if (!ros_dataset.HasNewData()) {
        loop_rate.sleep();
        continue;
      }
      
      cv::Mat image_left, image_right;
      double timestamp;
      ImuDataList batch_imu_data;
      
      // Get data from ROS topics (idx is ignored in ROS dataset)
      if (!ros_dataset.GetData(0, image_left, image_right, batch_imu_data, timestamp)) {
        loop_rate.sleep();
        continue;
      }
      
      std::cout << "Processing frame " << image_num << ", timestamp: " << std::fixed << timestamp << std::endl;
      
      InputDataPtr data = std::shared_ptr<InputData>(new InputData());
      data->index = image_num;
      data->time = timestamp;
      data->image_left = image_left;
      data->image_right = image_right;
      data->batch_imu_data = batch_imu_data;

      auto before_infer = std::chrono::high_resolution_clock::now();   
      map_builder.AddInput(data);
      auto after_infer = std::chrono::high_resolution_clock::now();
      auto cost_time = std::chrono::duration_cast<std::chrono::milliseconds>(after_infer - before_infer).count();
      sum_time += (double)cost_time;
      image_num++;
      std::cout << "One Frame Processing Time: " << cost_time << " ms." << std::endl;
      
      loop_rate.sleep();
    }
    
    if (image_num > 0) {
      std::cout << "Average FPS = " << image_num / (sum_time / 1000.0) << std::endl;
    }
    
  } else {
    ROS_INFO("=== USING FILE-BASED DATASET ===");
    
    // ORIGINAL: File-based dataset processing (commented but functional)
    Dataset dataset(configs.dataroot, map_builder.UseIMU());
    size_t dataset_length = dataset.GetDatasetLength();
    std::cout << "dataset done, length: " << dataset_length << std::endl;

    double sum_time = 0;
    int image_num = 0;
    
    // ORIGINAL: File-based processing loop
    for(size_t i = 0; i < dataset_length && ros::ok(); ++i){
      std::cout << "i ====== " << i << std::endl;
      cv::Mat image_left, image_right;
      double timestamp;
      ImuDataList batch_imu_data;
      if(!dataset.GetData(i, image_left, image_right, batch_imu_data, timestamp)) continue;

      InputDataPtr data = std::shared_ptr<InputData>(new InputData());
      data->index = i;
      data->time = timestamp;
      data->image_left = image_left;
      data->image_right = image_right;
      data->batch_imu_data = batch_imu_data;

      auto before_infer = std::chrono::high_resolution_clock::now();   
      map_builder.AddInput(data);
      auto after_infer = std::chrono::high_resolution_clock::now();
      auto cost_time = std::chrono::duration_cast<std::chrono::milliseconds>(after_infer - before_infer).count();
      sum_time += (double)cost_time;
      image_num++;
      std::cout << "One Frame Processing Time: " << cost_time << " ms." << std::endl;
    }
    
    if (image_num > 0) {
      std::cout << "Average FPS = " << image_num / (sum_time / 1000.0) << std::endl;
    }
  }

  std::cout << "Waiting to stop..." << std::endl; 
  map_builder.Stop();
  while(!map_builder.IsStopped()){
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  std::cout << "Map building has been stopped" << std::endl; 

  std::string trajectory_path = ConcatenateFolderAndFileName(configs.saving_dir, "trajectory_v0.txt");
  map_builder.SaveTrajectory(trajectory_path);
  map_builder.SaveMap(configs.saving_dir);
  ros::shutdown();

  return 0;
}
