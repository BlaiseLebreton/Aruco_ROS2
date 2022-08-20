#include "aruco_tracker.hpp"

ArucoTracker::ArucoTracker() : Node("aruco_tracker") {
  sub_image_ = image_transport::create_camera_subscription(this, "image", std::bind(&ArucoTracker::ImageCallback, this, std::placeholders::_1, std::placeholders::_2), "raw");
  publisher_ = this->create_publisher<sensor_msgs::msg::Image>("detection", rclcpp::SystemDefaultsQoS());

  dictionaryId = 0;
  estimatePose = false;
  markerLength = 0.07;

  dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));
  detectorParams = cv::aruco::DetectorParameters::create();

}

void ArucoTracker::ImageCallback(
  sensor_msgs::msg::Image::ConstSharedPtr image_msg,
  sensor_msgs::msg::CameraInfo::ConstSharedPtr info_msg) {

  cv::Mat image, imageCopy;
  cv::Mat camMatrix = cv::Mat(info_msg->k).reshape(1, 3);
  cv::Mat distCoeff = cv::Mat(info_msg->d).t();
  camMatrix.convertTo(camMatrix, CV_64F);
  distCoeff.convertTo(distCoeff, CV_64F);

  image = cv::imread("/workspaces/ros-ws/launch/aruco.png");

  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners, rejected;
  std::vector<cv::Vec3d> rvecs, tvecs;

  // Detect markers
  cv::aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);

  // Estimate their position
  if (ids.size() > 0) {
    cv::aruco::estimatePoseSingleMarkers(corners, markerLength, camMatrix, distCoeff, rvecs, tvecs);
  }

  // Draw on output image
  image.copyTo(imageCopy);
  if (ids.size() > 0) {
    cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);

    // Draw axis
    for (unsigned int i = 0; i < ids.size(); i++) {
      cv::aruco::drawAxis(imageCopy, camMatrix, distCoeff, rvecs[i], tvecs[i], markerLength * 2.5f);
    }
  }
  sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", imageCopy).toImageMsg();
  publisher_->publish(*msg.get());
}