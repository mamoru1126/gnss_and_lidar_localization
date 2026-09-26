#include "gll_ros2/localizer_node.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<gll_ros2::LocalizerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
