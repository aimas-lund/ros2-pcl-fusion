#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <memory>

namespace fusion
{

class FusionNode : public rclcpp::Node
{
public:
  explicit FusionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void sync_callback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & a,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & b);

  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloudA_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloudB_;
  std::shared_ptr<message_filters::Synchronizer<
    message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::PointCloud2,
      sensor_msgs::msg::PointCloud2>>> sync_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};

}  // namespace fusion
