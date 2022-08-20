#include "aruco_tracker.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArucoTracker>());
  rclcpp::shutdown();
  return 0;
}