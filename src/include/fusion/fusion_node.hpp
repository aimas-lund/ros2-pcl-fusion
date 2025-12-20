#ifndef FUSION__FUSION_NODE_HPP_
#define FUSION__FUSION_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

namespace fusion
{

class FusionNode : public rclcpp::Node
{
public:
  explicit FusionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void on_timer();

  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace fusion

#endif  // FUSION__FUSION_NODE_HPP_
