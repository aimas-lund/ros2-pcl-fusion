#include <memory>

#include "fusion/fusion_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<fusion::FusionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
