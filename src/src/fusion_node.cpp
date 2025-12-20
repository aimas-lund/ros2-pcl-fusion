#include "fusion/fusion_node.hpp"

#include <chrono>
#include <functional>

namespace fusion
{

using namespace std::chrono_literals;

FusionNode::FusionNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("fusion_node", options)
{
  auto period = std::chrono::milliseconds{1000};
  timer_ = this->create_wall_timer(period, std::bind(&FusionNode::on_timer, this));
}

void FusionNode::on_timer()
{
  RCLCPP_DEBUG(get_logger(), "Fusion node tick");
}

}  // namespace fusion
