#include <yesense_ros2_driver/yesense_ros2_driver.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImuSerialNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
