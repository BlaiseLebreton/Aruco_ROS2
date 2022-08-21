#include "aruco_tracker.hpp"

ArucoTracker::ArucoTracker() : Node("aruco_tracker") {
  tf_publisher_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  sub_image_    = image_transport::create_camera_subscription(this, "image", std::bind(&ArucoTracker::ImageCallback, this, std::placeholders::_1, std::placeholders::_2), "raw");
  pub_pos_      = this->create_publisher<geometry_msgs::msg::PoseArray>("poses",     rclcpp::SystemDefaultsQoS());
  pub_aru_      = this->create_publisher<aruco_msgs::msg::ArucoMarkerArray>("aruco",     rclcpp::SystemDefaultsQoS());

  dictionaryId = 0;
  estimatePose = false;
  markerLength = 0.07;

  dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));
  detectorParams = cv::aruco::DetectorParameters::create();

}

void ArucoTracker::ImageCallback(
  sensor_msgs::msg::Image::ConstSharedPtr image_msg,
  sensor_msgs::msg::CameraInfo::ConstSharedPtr info_msg) {
    aruco_msgs::msg::ArucoMarkerArray markers_detected = RunDetection(image_msg, info_msg);
    aruco_msgs::msg::ArucoMarkerArray markers_filtered = RunFilter(markers_detected);
    pub_aru_->publish(markers_filtered);
}

aruco_msgs::msg::ArucoMarkerArray ArucoTracker::RunDetection(
  sensor_msgs::msg::Image::ConstSharedPtr image_msg,
  sensor_msgs::msg::CameraInfo::ConstSharedPtr info_msg) {

  // Output
  aruco_msgs::msg::ArucoMarkerArray aruco_markers;
  aruco_markers.header.frame_id = image_msg->header.frame_id;

  // Load sensor info
  cv::Mat camMatrix = cv::Mat(info_msg->k).reshape(1, 3);
  cv::Mat distCoeff = cv::Mat(info_msg->d).t();
  camMatrix.convertTo(camMatrix, CV_64F);
  distCoeff.convertTo(distCoeff, CV_64F);

  // Load image
  cv::Mat image;
  image = cv::imread("/workspaces/ros-ws/launch/aruco.png");

  // Variables
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners, rejected;
  std::vector<cv::Vec3d> rvecs, tvecs;

  // Detect markers
  cv::aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);

  // Resize
  aruco_markers.markers.resize(ids.size());

  // Debug pose array
  geometry_msgs::msg::PoseArray pose_array;
  pose_array.header = aruco_markers.header;

  // Estimate their position
  if (ids.size() > 0) {
    cv::aruco::estimatePoseSingleMarkers(corners, markerLength, camMatrix, distCoeff, rvecs, tvecs);
    for (unsigned int i = 0; i < ids.size(); i++) {

      // Convert rvecs to quaternions
      cv::Mat R;
      tf2::Quaternion quat_tf;
      cv::Rodrigues(rvecs[i], R);
      tf2::Matrix3x3 tf2_rot(R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
                             R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
                             R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2));
      tf2::Transform tf2_transform(tf2_rot, tf2::Vector3());

      // Fill data
      aruco_markers.markers[i].id                 = ids[i];
      aruco_markers.markers[i].pose.position.x    = tvecs[i][0];
      aruco_markers.markers[i].pose.position.y    = tvecs[i][1];
      aruco_markers.markers[i].pose.position.z    = tvecs[i][2];
      aruco_markers.markers[i].pose.orientation.x = tf2_transform.getRotation().getX();
      aruco_markers.markers[i].pose.orientation.y = tf2_transform.getRotation().getY();
      aruco_markers.markers[i].pose.orientation.z = tf2_transform.getRotation().getZ();
      aruco_markers.markers[i].pose.orientation.w = tf2_transform.getRotation().getW();

      // Debug pose array
      pose_array.poses.push_back(aruco_markers.markers[i].pose);

    }
    pub_pos_->publish(pose_array);
  }

  return aruco_markers;
}

aruco_msgs::msg::ArucoMarkerArray ArucoTracker::RunFilter(aruco_msgs::msg::ArucoMarkerArray markers) {
  return markers;
}