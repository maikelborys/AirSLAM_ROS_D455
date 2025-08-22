#ifndef ROS_DATASET_H_
#define ROS_DATASET_H_

#include <vector>
#include <queue>
#include <mutex>
#include <Eigen/Dense>
#include <Eigen/SparseCore>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

// ROS includes
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "imu.h"
#include "utils.h"

struct StereoIMUData {
  cv::Mat left_image;
  cv::Mat right_image;
  double timestamp;
  ImuDataList imu_data;
  bool valid;
};

class ROSDataset {
public:
  ROSDataset(ros::NodeHandle& nh, bool use_imu);
  ~ROSDataset();
  
  // Interface compatible with original Dataset class
  size_t GetDatasetLength(); // Returns current buffer size
  bool GetData(size_t idx, cv::Mat& left_image, cv::Mat& right_image, ImuDataList& batch_imu_data, double& timestamp);
  
  // ROS-specific methods
  bool HasNewData();
  void ClearProcessedData();
  
private:
  // ROS subscribers and synchronizer
  ros::NodeHandle _nh;
  bool _use_imu;
  
  // Image subscribers with synchronization
  message_filters::Subscriber<sensor_msgs::Image> _left_image_sub;
  message_filters::Subscriber<sensor_msgs::Image> _right_image_sub;
  
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> StereoSyncPolicy;
  typedef message_filters::Synchronizer<StereoSyncPolicy> StereoSync;
  boost::shared_ptr<StereoSync> _stereo_sync;
  
  // IMU-only subscriber for when not using sync
  ros::Subscriber _imu_only_sub;
  
  // Data buffers
  std::queue<StereoIMUData> _data_buffer;
  std::queue<ImuData> _imu_buffer;
  std::mutex _buffer_mutex;
  std::mutex _imu_mutex;
  
  // Dataset-style stateful IMU processing (to match Dataset exactly)
  std::vector<ImuData> _imu_vector;  // Sequential IMU data for processing
  size_t _imu_idx;                   // Current IMU processing index (maintained across frames)
  std::mutex _imu_state_mutex;
  
  // Last processed data
  double _last_image_time;
  size_t _processed_count;
  
  // Topic names (configurable)
  std::string _left_image_topic;
  std::string _right_image_topic;
  std::string _imu_topic;
  
  // Callbacks
  void stereoCallback(const sensor_msgs::ImageConstPtr& left_msg, 
                     const sensor_msgs::ImageConstPtr& right_msg);
  void imuCallback(const sensor_msgs::ImuConstPtr& imu_msg);
  
  // Helper methods
  cv::Mat convertImageMsg(const sensor_msgs::ImageConstPtr& msg);
  double rosTimeToDouble(const ros::Time& time);
  ImuDataList getIMUDataForFrame(double last_image_time, double current_image_time);
};

#endif // ROS_DATASET_H_
