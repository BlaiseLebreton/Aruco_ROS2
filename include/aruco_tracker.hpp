#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "tf2/convert.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/buffer_interface.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "aruco_msgs/msg/aruco_marker_array.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include "KFilter/include/KFilter.h"

using namespace std::chrono_literals;


class ArucoTracker : public rclcpp::Node {
public:
  ArucoTracker();

private:
  void ImageCallback(
    sensor_msgs::msg::Image::ConstSharedPtr image_msg,
    sensor_msgs::msg::CameraInfo::ConstSharedPtr info_msg
  );

  aruco_msgs::msg::ArucoMarkerArray RunDetection(
    sensor_msgs::msg::Image::ConstSharedPtr image_msg,
    sensor_msgs::msg::CameraInfo::ConstSharedPtr info_msg
  );

  aruco_msgs::msg::ArucoMarkerArray RunFilter(aruco_msgs::msg::ArucoMarkerArray markers);

  // ROS Pub/Sub
  image_transport::CameraSubscriber sub_image_;
  rclcpp::Publisher<aruco_msgs::msg::ArucoMarkerArray>::SharedPtr pub_aru_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_pos_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_publisher_;

  // Parameters
  int dictionaryId;
  bool estimatePose;
  float markerLength;

  // Variables
  cv::Ptr<cv::aruco::Dictionary> dictionary;
  cv::Ptr<cv::aruco::DetectorParameters> detectorParams;
};
