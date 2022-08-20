#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>

using namespace std::chrono_literals;


class ArucoTracker : public rclcpp::Node {
public:
  ArucoTracker();

private:
  void ImageCallback(
    sensor_msgs::msg::Image::ConstSharedPtr image_msg,
    sensor_msgs::msg::CameraInfo::ConstSharedPtr info_msg
  );

  image_transport::CameraSubscriber sub_image_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  size_t count_;

  // Parameters
  int dictionaryId;
  bool estimatePose;
  float markerLength;

  // Variables
  cv::Ptr<cv::aruco::Dictionary> dictionary;
  cv::Ptr<cv::aruco::DetectorParameters> detectorParams;
};
