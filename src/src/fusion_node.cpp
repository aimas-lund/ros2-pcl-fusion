#include "fusion/fusion_node.hpp"

#include <functional>
#include <stdexcept>
#include <string>
#include <vector>


namespace fusion
{

FusionNode::FusionNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("fusion_node", options)
{
  auto params_ = handleParams();

  auto qos = rclcpp::SensorDataQoS();

  cloudA_.subscribe(this, params_.input_topics[0], qos.get_rmw_qos_profile());
  cloudB_.subscribe(this, params_.input_topics[1], qos.get_rmw_qos_profile());

  pub_ = this->create_publisher<std_msgs::msg::String>(params_.output_topic, params_.queue_size);

  sync_ = std::make_shared<
      message_filters::Synchronizer<
        message_filters::sync_policies::ApproximateTime<
          sensor_msgs::msg::PointCloud2,
          sensor_msgs::msg::PointCloud2
        >
      >
    >( message_filters::sync_policies::ApproximateTime<
         sensor_msgs::msg::PointCloud2,
         sensor_msgs::msg::PointCloud2>(params_.queue_size),
      cloudA_, cloudB_ );

  sync_->registerCallback(
    std::bind(&FusionNode::sync_callback, this, std::placeholders::_1, std::placeholders::_2)
  );
  sync_->setAgePenalty(params_.slop);
}

/**
 * Handle parameter declaration and retrieval
 */
FusionNode::FusionNodeParameters FusionNode::handleParams()
{
  const uint32_t default_queue_size = 10U;
  FusionNodeParameters params;

  this->declare_parameter<int>("message_sync.queue_size", static_cast<int>(default_queue_size));
  this->declare_parameter<double>("message_sync.slop", 0.1);
  this->declare_parameter<std::vector<std::string>>(
    "input_topics", {"/blueview/points2", "/blueview/points2"});
  this->declare_parameter<std::string>("output_topic", "/fused_pointcloud");

  const auto queue_size_param = this->get_parameter("message_sync.queue_size").as_int();
  params.slop = this->get_parameter("message_sync.slop").as_double();
  params.input_topics = this->get_parameter("input_topics").as_string_array();
  params.output_topic = this->get_parameter("output_topic").as_string();

  if (params.input_topics.size() < 2U) {
    RCLCPP_FATAL(this->get_logger(), "Expected two input topics; received %zu", params.input_topics.size());
    throw std::runtime_error("fusion_node requires exactly two input topics");
  }

  const auto queue_size = queue_size_param <= 0 ? 0U : static_cast<uint32_t>(queue_size_param);
  params.queue_size = queue_size == 0U ? default_queue_size : queue_size;
  if (queue_size == 0U) {
    RCLCPP_WARN(this->get_logger(), "Configured queue_size <= 0; defaulting to %u", params.queue_size);
  }

  RCLCPP_INFO(this->get_logger(), "FusionNode parameters:");
  RCLCPP_INFO(this->get_logger(), "  input_topics: [%s, %s]",
    params.input_topics[0].c_str(),
    params.input_topics[1].c_str());
  RCLCPP_INFO(this->get_logger(), "  output_topic: %s", params.output_topic.c_str());
  RCLCPP_INFO(this->get_logger(), "  message_sync.queue_size: %u", params.queue_size);
  RCLCPP_INFO(this->get_logger(), "  message_sync.slop: %f", params.slop);

  return params;
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

// sensor_msgs::msg::PointCloud2::ConstSharedPtr FusionNode::transformCloud(
//   const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pcl)
// {
//   // Merging logic to be implemented
//   return pcl;  // Placeholder
// }



}  // namespace fusion
