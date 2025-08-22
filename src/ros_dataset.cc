#include "ros_dataset.h"
#include <cv_bridge/cv_bridge.h>

ROSDataset::ROSDataset(ros::NodeHandle& nh, bool use_imu) 
  : _nh(nh), _use_imu(use_imu), _last_image_time(0.0), _processed_count(0), _imu_idx(0) {
  
  // Get topic names from parameters (with defaults)
  // Use private node handle to read parameters set on this node
  ros::NodeHandle private_nh("~");
  private_nh.param<std::string>("left_image_topic", _left_image_topic, "/cam0/image_raw");
  private_nh.param<std::string>("right_image_topic", _right_image_topic, "/cam1/image_raw");
  private_nh.param<std::string>("imu_topic", _imu_topic, "/imu0");
  
  ROS_INFO("ROSDataset: Subscribing to topics:");
  ROS_INFO("  Left image: %s", _left_image_topic.c_str());
  ROS_INFO("  Right image: %s", _right_image_topic.c_str());
  if (_use_imu) {
    ROS_INFO("  IMU: %s", _imu_topic.c_str());
  }
  
  // Set up stereo image synchronization (SIMPLE)
  _left_image_sub.subscribe(_nh, _left_image_topic, 10);
  _right_image_sub.subscribe(_nh, _right_image_topic, 10);
  
  _stereo_sync.reset(new StereoSync(StereoSyncPolicy(10), _left_image_sub, _right_image_sub));
  _stereo_sync->registerCallback(boost::bind(&ROSDataset::stereoCallback, this, _1, _2));
  
  // Set up IMU subscription
  if (_use_imu) {
    _imu_only_sub = _nh.subscribe(_imu_topic, 100, &ROSDataset::imuCallback, this);
  }
  
  ROS_INFO("ROSDataset: Initialization complete");
}

ROSDataset::~ROSDataset() {
  ROS_INFO("ROSDataset: Shutting down");
}

void ROSDataset::stereoCallback(const sensor_msgs::ImageConstPtr& left_msg, 
                               const sensor_msgs::ImageConstPtr& right_msg) {
  
  // Convert ROS images to OpenCV
  cv::Mat left_image = convertImageMsg(left_msg);
  cv::Mat right_image = convertImageMsg(right_msg);
  
  if (left_image.empty() || right_image.empty()) {
    ROS_WARN("ROSDataset: Failed to convert images");
    return;
  }
  
  // Get timestamp (use left image timestamp as reference)
  double timestamp = rosTimeToDouble(left_msg->header.stamp);
  
    // Pre-associate IMU data using Dataset-style logic
  ImuDataList imu_data;
  if (_use_imu) {
    // Get IMU data between last frame and current frame (Dataset-style)
    imu_data = getIMUDataForFrame(_last_image_time, timestamp);
  } else {
    ROS_INFO_THROTTLE(5.0, "ROSDataset: IMU disabled in config");
  }
  
  // Create stereo+IMU data package with pre-associated IMU data
  StereoIMUData data;
  data.left_image = left_image.clone();
  data.right_image = right_image.clone();
  data.timestamp = timestamp;
  data.imu_data = imu_data;  // Store pre-associated IMU data!
  data.valid = true;
  
  // Add to buffer (SIMPLE)
  {
    std::lock_guard<std::mutex> lock(_buffer_mutex);
    _data_buffer.push(data);
    
    // Keep buffer small 
    while (_data_buffer.size() > 5) {
      _data_buffer.pop();
    }
  }
  
  _last_image_time = timestamp;
  
  // Debug output
  static int count = 0;
  if (++count % 30 == 0) { // Print every 30 frames
    ROS_INFO("ROSDataset: Received %d stereo pairs, buffer size: %lu", 
             count, _data_buffer.size());
  }
}

void ROSDataset::imuCallback(const sensor_msgs::ImuConstPtr& imu_msg) {
  if (!_use_imu) return;
  
  // Convert ROS IMU to AirSLAM IMU format
  ImuData imu_data;
  imu_data.timestamp = rosTimeToDouble(imu_msg->header.stamp);
  // FIXED: Use correct field names (gyr, acc instead of angular_velocity, linear_acceleration)
  imu_data.gyr.x() = imu_msg->angular_velocity.x;
  imu_data.gyr.y() = imu_msg->angular_velocity.y;
  imu_data.gyr.z() = imu_msg->angular_velocity.z;
  imu_data.acc.x() = imu_msg->linear_acceleration.x;
  imu_data.acc.y() = imu_msg->linear_acceleration.y;
  imu_data.acc.z() = imu_msg->linear_acceleration.z;
  
  // Debug: Log IMU data reception
  static int imu_count = 0;
  if (++imu_count % 100 == 0) {
    ROS_INFO("ROSDataset: Received %d IMU samples, latest: gyr=(%.3f,%.3f,%.3f), acc=(%.3f,%.3f,%.3f)", 
             imu_count, imu_data.gyr.x(), imu_data.gyr.y(), imu_data.gyr.z(),
             imu_data.acc.x(), imu_data.acc.y(), imu_data.acc.z());
  }
  
  // Add to IMU buffer (for compatibility)
  {
    std::lock_guard<std::mutex> lock(_imu_mutex);
    _imu_buffer.push(imu_data);
    
    // Keep IMU buffer small
    while (_imu_buffer.size() > 50) {
      _imu_buffer.pop();
    }
  }
  
  // Add to Dataset-style IMU vector for stateful processing
  {
    std::lock_guard<std::mutex> lock(_imu_state_mutex);
    _imu_vector.push_back(imu_data);
    
    // Keep vector manageable but larger than buffer for better association
    if (_imu_vector.size() > 1000) {
      // Remove oldest samples, but preserve _imu_idx validity
      size_t remove_count = 200;
      if (_imu_idx >= remove_count) {
        _imu_idx -= remove_count;
      } else {
        _imu_idx = 0;
      }
      _imu_vector.erase(_imu_vector.begin(), _imu_vector.begin() + remove_count);
    }
  }
}

cv::Mat ROSDataset::convertImageMsg(const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    // Try to convert to mono8 (grayscale)
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    return cv_ptr->image;
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("ROSDataset: cv_bridge exception: %s", e.what());
    return cv::Mat();
  }
}

double ROSDataset::rosTimeToDouble(const ros::Time& time) {
  return time.sec + time.nsec * 1e-9;
}

// EXACT Dataset logic - maintains state across frame processing like Dataset class
ImuDataList ROSDataset::getIMUDataForFrame(double last_image_time, double current_image_time) {
  ImuDataList result;
  if (!_use_imu) return result;
  
  std::lock_guard<std::mutex> lock(_imu_state_mutex);
  
  if (_imu_vector.empty()) {
    return result;
  }
  
  // Apply EXACT Dataset algorithm with maintained imu_idx state:
  // for(; imu_idx < all_imu_data.size()-1; imu_idx++){
  //   if(all_imu_data[imu_idx+1].timestamp < last_image_time) continue;
  //   mini_batch_imu_data.emplace_back(all_imu_data[imu_idx]);
  //   if(all_imu_data[imu_idx].timestamp > image_time) break;
  // }
  // imu_idx--;
  
  for (; _imu_idx < _imu_vector.size() - 1; ++_imu_idx) {
    // Skip IMU data that's too old (before last frame)
    if (_imu_idx < _imu_vector.size() - 1 && _imu_vector[_imu_idx + 1].timestamp < last_image_time) {
      continue;
    }
    
    // Add this IMU sample to the result
    result.push_back(_imu_vector[_imu_idx]);
    
    // Stop when we reach data after current image time
    if (_imu_vector[_imu_idx].timestamp > current_image_time) {
      break;
    }
  }
  
  // Backtrack for next frame (Dataset-style)
  if (_imu_idx > 0) {
    _imu_idx--;
  }
  
  return result;
}

size_t ROSDataset::GetDatasetLength() {
  std::lock_guard<std::mutex> lock(_buffer_mutex);
  return _data_buffer.size();
}

bool ROSDataset::GetData(size_t idx, cv::Mat& left_image, cv::Mat& right_image, 
                        ImuDataList& batch_imu_data, double& timestamp) {
  
  std::lock_guard<std::mutex> lock(_buffer_mutex);
  
  // For ROS dataset, we use FIFO queue instead of random access
  // The 'idx' parameter is ignored, we always process the next available data
  if (_data_buffer.empty()) {
    return false; // No data available
  }
  
  // Get the front data (oldest unprocessed)
  StereoIMUData data = _data_buffer.front();
  _data_buffer.pop();
  
  if (!data.valid) {
    return false;
  }
  
  // Return the data
  left_image = data.left_image;
  right_image = data.right_image;
  batch_imu_data = data.imu_data;
  timestamp = data.timestamp;
  
  _processed_count++;
  
  return true;
}

bool ROSDataset::HasNewData() {
  std::lock_guard<std::mutex> lock(_buffer_mutex);
  return !_data_buffer.empty();
}

void ROSDataset::ClearProcessedData() {
  // This method can be used to clear old processed data if needed
  // For now, we handle this automatically in the callbacks
}
