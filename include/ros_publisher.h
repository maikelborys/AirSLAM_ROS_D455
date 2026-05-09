#ifndef ROS_PUBLISHER_H_
#define ROS_PUBLISHER_H_

#include <map>
#include <vector>
#include <unordered_map>
#include <memory>

#include <opencv2/opencv.hpp>
#include <Eigen/Core>

// AirSLAM Jazzy port: ROS 1 publishers + tf::TransformBroadcaster replaced
// by their rclcpp / tf2_ros equivalents.
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "utils.h"
#include "read_configs.h"
#include "thread_publisher.h"

enum FeatureMessgaeType {
  VOFeature = 0,
  RelocFeature = 1
};

struct FeatureMessgae{
  double time;
  cv::Mat image;
  cv::Mat key_image;
  int frame_id;
  int keyfrmae_id;
  std::vector<bool> inliers;
  std::vector<cv::KeyPoint> keyframe_keypoints;
  std::vector<cv::KeyPoint> keypoints;
  std::vector<Eigen::Vector4d> lines;
  std::vector<int> line_track_ids;
  std::vector<std::map<int, double>> points_on_lines;
  std::vector<cv::DMatch> matches;
  FeatureMessgaeType fm_type;
};
typedef std::shared_ptr<FeatureMessgae> FeatureMessgaePtr;
typedef std::shared_ptr<const FeatureMessgae> FeatureMessgaeConstPtr;

struct FramePoseMessage{
  double time;
  Eigen::Matrix4d pose;
};
typedef std::shared_ptr<FramePoseMessage> FramePoseMessagePtr;
typedef std::shared_ptr<const FramePoseMessage> FramePoseMessageConstPtr;

struct KeyframeMessage{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double time;
  std::vector<double> times;
  std::vector<int> ids;
  std::vector<Eigen::Matrix4d> poses;
};
typedef std::shared_ptr<KeyframeMessage> KeyframeMessagePtr;
typedef std::shared_ptr<const KeyframeMessage> KeyframeMessageConstPtr;

struct MapMessage{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double time;
  bool reset;
  std::vector<int> ids;
  std::vector<Eigen::Vector3d> points;
};
typedef std::shared_ptr<MapMessage> MapMessagePtr;
typedef std::shared_ptr<const MapMessage> MapMessageConstPtr;

struct MapLineMessage{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double time;
  bool reset;
  std::vector<int> ids;
  std::vector<Vector6d> lines;
};
typedef std::shared_ptr<MapLineMessage> MapLineMessagePtr;
typedef std::shared_ptr<const MapLineMessage> MapLineMessageConstPtr;


struct RelocMessage{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double map_scale;
  std::vector<double> times;
  std::vector<Eigen::Matrix4d> poses;
  std::vector<Eigen::Vector3d> mappoints;
};
typedef std::shared_ptr<RelocMessage> RelocMessagePtr;
typedef std::shared_ptr<const RelocMessage> RelocMessageConstPtr;


class RosPublisher{
public:
  RosPublisher(const RosPublisherConfig& ros_publisher_config,
               rclcpp::Node::SharedPtr nh);

  void PublishFeature(FeatureMessgaePtr feature_message);
  void PublishFramePose(FramePoseMessagePtr frame_pose_message);
  void PublisheKeyframe(KeyframeMessagePtr keyframe_message);
  void PublishMap(MapMessagePtr map_message);
  void PublishMapLine(MapLineMessagePtr mapline_message);
  void PubRelocResults(RelocMessagePtr reloc_message);

  void Clear();
  void ShutDown();

private:
  RosPublisherConfig _config;
  rclcpp::Node::SharedPtr _nh;

  // for publishing features
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _ros_feature_pub;
  ThreadPublisher<FeatureMessgae> _feature_publisher;

  // for publishing frame
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _ros_frame_pose_pub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr _pub_latest_odometry;
  std::unique_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster;
  ThreadPublisher<FramePoseMessage> _frame_pose_publisher;

  // for publishing keyframes
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr _ros_keyframe_pub;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr _ros_path_pub;
  std::map<int, int> _keyframe_id_to_index;
  geometry_msgs::msg::PoseArray  _ros_keyframe_array;
  nav_msgs::msg::Path _ros_path;
  ThreadPublisher<KeyframeMessage> _keyframe_publisher;

  // for publishing mappoints
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr _ros_map_pub;
  std::unordered_map<int, int> _mappoint_id_to_index;
  sensor_msgs::msg::PointCloud _ros_mappoints;
  ThreadPublisher<MapMessage> _map_publisher;

  // for publishing maplines
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr _ros_mapline_pub;
  std::unordered_map<int, int> _mapline_id_to_index;
  visualization_msgs::msg::Marker _ros_maplines;
  ThreadPublisher<MapLineMessage> _mapline_publisher;

  // for publishing relocalization results
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr _ros_reloc_traj_pub;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _ros_reloc_pose_pub;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr _ros_reloc_mpts_pub;
  visualization_msgs::msg::Marker _ros_reloc_traj;
  ThreadPublisher<RelocMessage> _reloc_traj_publisher;
  ThreadPublisher<RelocMessage> _reloc_pose_publisher;
  ThreadPublisher<RelocMessage> _reloc_mpts_publisher;
};
typedef std::shared_ptr<RosPublisher> RosPublisherPtr;

#endif  // ROS_PUBLISHER_H_
