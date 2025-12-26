#include <memory>
#include "fusion/fusion_node.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;

  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<fusion::FusionNode>(options);
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(node);
  executor->spin();
  
  rclcpp::shutdown();
  return 0;
}
