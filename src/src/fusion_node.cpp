#include "fusion/fusion_node.hpp"

#include "fusion/utils.hpp"

#include <cstring>
#include <functional>
#include <limits>
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

  pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    params_.output.topic, qos);

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
  const auto [slop_sec, slop_nsec] = params_.input.sync.slop;
  sync_->setMaxIntervalDuration(rclcpp::Duration(slop_sec, slop_nsec));
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
    declare_if_needed(
      "input.sync.slop_ns",
      rclcpp::ParameterValue(static_cast<int64_t>(100000000LL)));
    declare_if_needed(
      "input.topics",
      rclcpp::ParameterValue(std::vector<std::string>{"/cloud1", "/cloud2"}));
    declare_if_needed(
      "input.frame_ids",
      rclcpp::ParameterValue(std::vector<std::string>{"frame1", "frame2"}));
    declare_if_needed(
      "output.topic",
      rclcpp::ParameterValue(std::string{"/fusion/out"}));
    declare_if_needed(
      "output.frame_id",
      rclcpp::ParameterValue(std::string{"fused_frame"}));
    declare_if_needed(
      "transform.type",
      rclcpp::ParameterValue(std::string{"static"}));

    const auto queue_size_param = this->get_parameter("input.sync.queue_size").as_int();
    const auto slop_ns = this->get_parameter("input.sync.slop_ns").as_int();
    {
      const auto sec = static_cast<int32_t>(slop_ns / 1000000000LL);
      const auto nsec = static_cast<uint32_t>(slop_ns % 1000000000LL);
      params.input.sync.slop = std::make_tuple(sec, nsec);
    }
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
    RCLCPP_INFO(this->get_logger(), "  input.sync.slop_ns: %ld", static_cast<long>(slop_ns));
    RCLCPP_INFO(this->get_logger(), "  output.topic: %s", params.output.topic.c_str());
    RCLCPP_INFO(this->get_logger(), "  output.frame_id: %s", params.output.frame_id.c_str());
    RCLCPP_INFO(this->get_logger(), "  transform.type: %u", static_cast<uint32_t>(params.transform.type));
    
    return params;
  } catch (const rclcpp::ParameterTypeException & ex) {
    RCLCPP_FATAL(this->get_logger(), "Parameter type error: %s", ex.what());
    throw;
  }
}

/**
 * Callback for synchronized point clouds
 * @param a First point cloud
 * @param b Second point cloud
 */
void FusionNode::syncCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr &a,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr &b) 
{
  auto fused_cloud = fuse(a, b);
  if (!fused_cloud) {
    is_transmitting_ = false;
    RCLCPP_WARN(this->get_logger(), "Fusion failed. Skipped publishing fused cloud message.");
    return;
  }
  pub_->publish(std::move(fused_cloud));
  if (!is_transmitting_) {
    RCLCPP_INFO(this->get_logger(), "Started transmitting fused point clouds...");
    is_transmitting_ = true;
  }
}

/** 
 * Lookup transform from source frame to output frame
 * Caches static transforms for efficiency
 * @param source_frame The source frame to transform from
 * @return The TransformStamped from source_frame to output frame
*/
geometry_msgs::msg::TransformStamped FusionNode::lookupTransformToOutputFrame(
  const std::string & source_frame)
{
  if (params_.transform.type == TransformType::STATIC) {
    const auto it = static_tf_cache_.find(source_frame);
    if (it != static_tf_cache_.end()) {
      return it->second;
    }

    auto transform = tf_buffer_.lookupTransform(
      params_.output.frame_id,
      source_frame,
      tf2::TimePointZero);
    static_tf_cache_.emplace(source_frame, transform);
    return transform;
  }

  return tf_buffer_.lookupTransform(
    params_.output.frame_id,
    source_frame,
    tf2::TimePointZero);
}

/**
 * Transform point cloud to output frame
 * @param cloud The input point cloud
 * @return The transformed point cloud in the output frame
 */
sensor_msgs::msg::PointCloud2::ConstSharedPtr FusionNode::transformToOutputFrame(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud)
{
  if (!cloud) {
    return {};
  }

  if (cloud->header.frame_id == params_.output.frame_id) {
    return cloud;
  }

  try {
    const auto transform = lookupTransformToOutputFrame(cloud->header.frame_id);

    auto transformed = std::make_shared<sensor_msgs::msg::PointCloud2>();
    tf2::doTransform(*cloud, *transformed, transform);
    transformed->header.frame_id = params_.output.frame_id;
    transformed->header.stamp = transform.header.stamp;
    return transformed;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(),
      "Failed to transform cloud from '%s' to '%s': %s",
      cloud->header.frame_id.c_str(),
      params_.output.frame_id.c_str(),
      ex.what());
    return {};
  }
}

/**
 * Check if two PointCloud2 messages have matching layouts
 * @param a First point cloud
 * @param b Second point cloud
 * @return True if layouts match, false otherwise
 */
static bool hasMatchingPointCloud2Layout(
  const sensor_msgs::msg::PointCloud2 & a,
  const sensor_msgs::msg::PointCloud2 & b)
{
  if (a.point_step != b.point_step) {
    return false;
  }
  if (a.is_bigendian != b.is_bigendian) {
    return false;
  }
  if (a.fields.size() != b.fields.size()) {
    return false;
  }
  for (size_t i = 0; i < a.fields.size(); ++i) {
    const auto & fa = a.fields[i];
    const auto & fb = b.fields[i];
    if (fa.name != fb.name || fa.offset != fb.offset || fa.datatype != fb.datatype || fa.count != fb.count) {
      return false;
    }
  }
  return true;
}

/**
 * Fuse two point clouds into one
 * @param a First point cloud
 * @param b Second point cloud
 * @return The fused point cloud
 */
sensor_msgs::msg::PointCloud2::UniquePtr FusionNode::fuse(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & a,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & b)
{
  const auto transformed_a = transformToOutputFrame(a);
  const auto transformed_b = transformToOutputFrame(b);

  if (!transformed_a || !transformed_b) {
    return {};
  }

  const bool areFramesMismatched = transformed_a->header.frame_id != params_.output.frame_id ||
      transformed_b->header.frame_id != params_.output.frame_id;
  if (areFramesMismatched) {
    RCLCPP_WARN(this->get_logger(), "Transformed clouds are not in output frame.");
    return {};
  }

  if (!hasMatchingPointCloud2Layout(*transformed_a, *transformed_b)) {
    RCLCPP_WARN(this->get_logger(), "PointCloud2 layouts differ. Cannot fast-concatenate");
    return {};
  }

  const size_t step = transformed_a->point_step;
  const bool isPointStepInvalid = step == 0U;
  if (isPointStepInvalid) {
    RCLCPP_WARN(this->get_logger(), "Cannot fuse PointCloud2 with point_step of zero");
    return {};
  }

  const size_t bytes_a = transformed_a->data.size();
  const size_t bytes_b = transformed_b->data.size();
  const bool isDataSizeInvalid = (bytes_a % step != 0U) || (bytes_b % step != 0U);
  if (isDataSizeInvalid) {
    RCLCPP_WARN(this->get_logger(), "Cannot fuse PointCloud2 with data size not divisible by point_step");
    return {};
  }

  const size_t points_a = bytes_a / step;
  const size_t points_b = bytes_b / step;
  const bool isPointCountOverflowing = points_a > static_cast<size_t>(std::numeric_limits<uint32_t>::max()) ||
      points_b > static_cast<size_t>(std::numeric_limits<uint32_t>::max()) ||
      (points_a + points_b) > static_cast<size_t>(std::numeric_limits<uint32_t>::max());
  if (isPointCountOverflowing) {
    RCLCPP_WARN(this->get_logger(), "Cannot fuse as total point count exceeds uint32_t max");
    return {};
  }

  if (bytes_a > (std::numeric_limits<size_t>::max() - bytes_b)) {
    RCLCPP_WARN(this->get_logger(), "Cannot fuse as fused data size overflows size_t");
    return {};
  }

  const size_t fused_points = points_a + points_b;
  if (fused_points != 0U && step > (std::numeric_limits<size_t>::max() / fused_points)) {
    RCLCPP_WARN(this->get_logger(), "Cannot fuse as fused row_step overflows size_t");
    return {};
  }

  const size_t fused_row_step = fused_points * step;
  if (fused_row_step > static_cast<size_t>(std::numeric_limits<uint32_t>::max())) {
    RCLCPP_WARN(this->get_logger(), "Cannot fuse as fused row_step exceeds uint32_t max");
    return {};
  }

  const size_t fused_bytes = bytes_a + bytes_b;
  if (fused_bytes != fused_row_step) {
    RCLCPP_WARN(this->get_logger(), "Cannot fuse as fused data size does not match fused row_step");
    return {};
  }

  auto fused = std::make_unique<sensor_msgs::msg::PointCloud2>();

  fused->header.frame_id = params_.output.frame_id;
  fused->header.stamp = (rclcpp::Time(transformed_a->header.stamp) >= rclcpp::Time(transformed_b->header.stamp))
                          ? transformed_a->header.stamp
                          : transformed_b->header.stamp;

  fused->fields = transformed_a->fields;
  fused->is_bigendian = transformed_a->is_bigendian;
  fused->point_step = transformed_a->point_step;
  fused->is_dense = transformed_a->is_dense && transformed_b->is_dense;

  fused->height = 1U;
  fused->width = static_cast<uint32_t>(fused_points);
  fused->row_step = static_cast<uint32_t>(fused_row_step);

  fused->data.resize(fused_bytes);
  if (bytes_a != 0U) {
    std::memcpy(fused->data.data(), transformed_a->data.data(), bytes_a);
  }
  if (bytes_b != 0U) {
    std::memcpy(fused->data.data() + bytes_a, transformed_b->data.data(), bytes_b);
  }

  fused->header.stamp = this->get_clock()->now();

  return fused;
}

}  // namespace fusion
