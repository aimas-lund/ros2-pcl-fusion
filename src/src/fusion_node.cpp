#include "fusion/fusion_node.hpp"

#include "fusion/utils.hpp"

#include <functional>
#include <stdexcept>
#include <string>
#include <vector>
#include <tf2/exceptions.h>


namespace fusion
{

FusionNode::FusionNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("fusion_node", options),
  tf_buffer_(this->get_clock()),
  tf_listener_(std::make_shared<tf2_ros::TransformListener>(tf_buffer_))
{
  params_ = handleParams();

  auto qos = rclcpp::SensorDataQoS();

  cloudA_.subscribe(this, params_.input.topics[0], qos.get_rmw_qos_profile());
  cloudB_.subscribe(this, params_.input.topics[1], qos.get_rmw_qos_profile());

  pub_ = this->create_publisher<std_msgs::msg::String>(
    params_.output.topic,
    params_.input.sync.queue_size);

  sync_ = std::make_shared<
      message_filters::Synchronizer<
        message_filters::sync_policies::ApproximateTime<
          sensor_msgs::msg::PointCloud2,
          sensor_msgs::msg::PointCloud2
        >
      >
    >( message_filters::sync_policies::ApproximateTime<
         sensor_msgs::msg::PointCloud2,
          sensor_msgs::msg::PointCloud2>(params_.input.sync.queue_size),
      cloudA_, cloudB_ );

  sync_->registerCallback(
    std::bind(&FusionNode::syncCallback, this, std::placeholders::_1, std::placeholders::_2)
  );
  sync_->setAgePenalty(params_.input.sync.slop);
}

/**
 * Handle parameter declaration and retrieval
 * @return FusionNodeParameters struct containing the parsed node parameters
 */
FusionNodeParameters FusionNode::handleParams()
{
  const uint32_t default_queue_size = 10U;
  FusionNodeParameters params{};

  try {
    auto declare_if_needed =
      [&](const std::string & name, const rclcpp::ParameterValue & default_value) {
        if (!this->has_parameter(name)) {
          this->declare_parameter(name, default_value);
        }
      };

    declare_if_needed(
      "input.sync.queue_size",
      rclcpp::ParameterValue(static_cast<int>(default_queue_size)));
    declare_if_needed("input.sync.slop", rclcpp::ParameterValue(0.1));
    declare_if_needed(
      "input.topics",
      rclcpp::ParameterValue(std::vector<std::string>{"points1", "points2"}));
    declare_if_needed(
      "input.frame_ids",
      rclcpp::ParameterValue(std::vector<std::string>{"frame1", "frame2"}));
    declare_if_needed(
      "output.topic",
      rclcpp::ParameterValue(std::string{"/fused_pointcloud"}));
    declare_if_needed(
      "output.frame_id",
      rclcpp::ParameterValue(std::string{"fused_frame"}));
    declare_if_needed(
      "transform.type",
      rclcpp::ParameterValue(std::string{"static"}));

    const auto queue_size_param = this->get_parameter("input.sync.queue_size").as_int();
    params.input.sync.slop = this->get_parameter("input.sync.slop").as_double();
    params.input.topics = this->get_parameter("input.topics").as_string_array();
    params.input.frame_ids = this->get_parameter("input.frame_ids").as_string_array();
    params.output.topic = this->get_parameter("output.topic").as_string();
    params.output.frame_id = this->get_parameter("output.frame_id").as_string();

    const auto transform_type_str = this->get_parameter("transform.type").as_string();
    try {
      params.transform.type = parseTransformType(transform_type_str);
    } catch (const std::invalid_argument & ex) {
      RCLCPP_FATAL(
        this->get_logger(),
        "Invalid transform.type '%s' (expected 'static' or 'dynamic'): %s",
        transform_type_str.c_str(),
        ex.what());
      throw std::runtime_error("Invalid transform.type");
    }

    if (params.input.topics.size() != 2U) {
      RCLCPP_FATAL(
        this->get_logger(),
        "Expected exactly two input topics; received %zu",
        params.input.topics.size());
      throw std::runtime_error("fusion_node requires exactly two input topics");
    }

    if (params.input.frame_ids.size() != params.input.topics.size()) {
      RCLCPP_FATAL(
        this->get_logger(),
        "Expected input.frame_ids to have the same length as input.topics (%zu); received %zu",
        params.input.topics.size(),
        params.input.frame_ids.size());
      throw std::runtime_error("input.frame_ids length must match input.topics length");
    }

    const auto queueSize = queue_size_param <= 0 ? 0U : static_cast<uint32_t>(queue_size_param);
    params.input.sync.queue_size = queueSize == 0U ? default_queue_size : queueSize;
    if (queueSize == 0U) {
      RCLCPP_WARN(
        this->get_logger(),
        "Configured input.sync.queue_size <= 0; defaulting to %u",
        params.input.sync.queue_size);
    }

    RCLCPP_INFO(this->get_logger(), "FusionNode parameters:");
    RCLCPP_INFO(
      this->get_logger(),
      "  input.topics: [%s, %s]",
      params.input.topics[0].c_str(),
      params.input.topics[1].c_str());
    RCLCPP_INFO(
      this->get_logger(),
      "  input.frame_ids: [%s, %s]",
      params.input.frame_ids[0].c_str(),
      params.input.frame_ids[1].c_str());
    RCLCPP_INFO(this->get_logger(), "  input.sync.queue_size: %u", params.input.sync.queue_size);
    RCLCPP_INFO(this->get_logger(), "  input.sync.slop: %f", params.input.sync.slop);
    RCLCPP_INFO(this->get_logger(), "  output.topic: %s", params.output.topic.c_str());
    RCLCPP_INFO(this->get_logger(), "  output.frame_id: %s", params.output.frame_id.c_str());
    RCLCPP_INFO(this->get_logger(), "  transform.type: %u", static_cast<uint32_t>(params.transform.type));
    

    return params;
  } catch (const rclcpp::ParameterTypeException & ex) {
    RCLCPP_FATAL(this->get_logger(), "Parameter type error: %s", ex.what());
    throw;
  }
}

void FusionNode::syncCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr &a,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr &b) 
{
  RCLCPP_INFO(this->get_logger(), "Synchronized clouds received: A timestamp %u, B timestamp %u", a->header.stamp.sec, b->header.stamp.sec);

  const auto transformed_a = transformToOutputFrame(*a);
  const auto transformed_b = transformToOutputFrame(*b);

  auto message = std_msgs::msg::String();
  message.data = "Synced Data!";
  pub_->publish(message);
  (void)transformed_a;
  (void)transformed_b;
}

sensor_msgs::msg::PointCloud2 FusionNode::transformToOutputFrame(
  const sensor_msgs::msg::PointCloud2 & cloud)
{
  if (cloud.header.frame_id == params_.output.frame_id) {
    return cloud;
  }

  try {
    const auto transform = tf_buffer_.lookupTransform(
      params_.output.frame_id,
      cloud.header.frame_id,
      tf2::TimePointZero);

    sensor_msgs::msg::PointCloud2 transformed;
    tf2::doTransform(cloud, transformed, transform);
    transformed.header.frame_id = params_.output.frame_id;
    transformed.header.stamp = transform.header.stamp;
    return transformed;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(),
      "Failed to transform cloud from '%s' to '%s': %s",
      cloud.header.frame_id.c_str(),
      params_.output.frame_id.c_str(),
      ex.what());
    return cloud;
  }
}



}  // namespace fusion
