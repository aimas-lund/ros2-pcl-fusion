#include "fusion/fusion_node.hpp"

#include <chrono>
#include <functional>


namespace fusion
{

FusionNode::FusionNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("fusion_node", options)
{
  auto qos = rclcpp::SensorDataQoS();
  const uint32_t queue_size = 10;
  const double slop = 0.1;

  cloudA_.subscribe(this, "/cloudA", qos.get_rmw_qos_profile());
  cloudB_.subscribe(this, "/cloudB", qos.get_rmw_qos_profile());

  pub_ = this->create_publisher<std_msgs::msg::String>("/test/out", 10);

  sync_ = std::make_shared<
      message_filters::Synchronizer<
        message_filters::sync_policies::ApproximateTime<
          sensor_msgs::msg::PointCloud2,
          sensor_msgs::msg::PointCloud2
        >
      >
    >( message_filters::sync_policies::ApproximateTime<
         sensor_msgs::msg::PointCloud2,
         sensor_msgs::msg::PointCloud2>(queue_size),
      cloudA_, cloudB_ );

  sync_->registerCallback(
    std::bind(&FusionNode::sync_callback, this, std::placeholders::_1, std::placeholders::_2)
  );
  sync_->setAgePenalty(slop);
}

void FusionNode::sync_callback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr &a,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr &b) 
{
  RCLCPP_INFO(this->get_logger(), "Synchronized clouds received: A timestamp %u, B timestamp %u", a->header.stamp.sec, b->header.stamp.sec);
  
  auto message = std_msgs::msg::String();
  message.data = "Synced Data!";
  pub_->publish(message);
  // do something else
}

}  // namespace fusion
