#include "fusion/fusion_node.hpp"

#include "fusion/utils.hpp"


#include <cstring>
#include <Eigen/Geometry>
#include <functional>
#include <limits>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <stdexcept>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>


namespace fusion
{


FusionNode::FusionNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("fusion_node", options),
  tf_buffer_(this->get_clock()),
  tf_listener_(std::make_shared<tf2_ros::TransformListener>(tf_buffer_)),
  stop_worker_(false)
{
  params_ = handleParams();

  auto qos = rclcpp::SensorDataQoS().keep_last(
    static_cast<size_t>(std::max(params_.input.sync.queue_size, 10U))
  );

  cloudA_.subscribe(this, params_.input.topics[0], qos.get_rmw_qos_profile());
  cloudB_.subscribe(this, params_.input.topics[1], qos.get_rmw_qos_profile());

  pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    params_.output.topic, qos);

  sync_ = std::make_shared<
    message_filters::Synchronizer<FusionSyncPolicy>>(
    FusionSyncPolicy(params_.input.sync.queue_size, rclcpp::Duration::from_nanoseconds(params_.input.sync.epsilon_ms * 1000000ULL)),
    cloudA_, cloudB_ );

  sync_->registerCallback(
    std::bind(&FusionNode::syncCallback, this, std::placeholders::_1, std::placeholders::_2)
  );

  worker_thread_ = std::thread([this]() { this->workerLoop(); });
}

FusionNode::~FusionNode() {
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    stop_worker_ = true;
  }
  queue_cv_.notify_one();
  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }
}

void FusionNode::workerLoop() {
  while (true) {
    SyncedData data;
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      queue_cv_.wait(lock, [this]() { return !sync_queue_.empty() || stop_worker_; });
      if (stop_worker_ && sync_queue_.empty()) {
        break;
      }
      data = std::move(sync_queue_.front());
      sync_queue_.pop();
    }
    processSyncData(data.a, data.b);
  }
}

void FusionNode::processSyncData(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &a,
                                 const sensor_msgs::msg::PointCloud2::ConstSharedPtr &b) {
  auto fused_cloud = this->fuse(a, b);
  if (!fused_cloud) {
    std::lock_guard<std::mutex> lock(transmit_mutex_);
    is_transmitting_ = false;
    RCLCPP_WARN(this->get_logger(), "Fusion failed. Skipped publishing fused cloud message.");
    return;
  }
  pub_->publish(std::move(fused_cloud));
  {
    std::lock_guard<std::mutex> lock(transmit_mutex_);
    if (!is_transmitting_) {
      RCLCPP_INFO(this->get_logger(), "Started transmitting fused point clouds...");
      is_transmitting_ = true;
    }
  }
}

/**
 * Handle parameter declaration and retrieval
 * @return FusionNodeParameters struct containing the parsed node parameters
 */
FusionNodeParameters FusionNode::handleParams()
{
  const uint32_t default_queue_size = 10U;
  const int64_t default_slop_ms = 100LL;
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
      rclcpp::ParameterValue(static_cast<int64_t>(default_queue_size)));
    declare_if_needed(
      "input.sync.slop_ms",
      rclcpp::ParameterValue(static_cast<int64_t>(default_slop_ms)));
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
    const auto epsilon_ms_param = this->get_parameter("input.sync.epsilon_ms").as_int();

    auto epsilon_ms_used = epsilon_ms_param;
    if (epsilon_ms_used < 0LL) {
      RCLCPP_WARN(this->get_logger(), "Configured input.sync.epsilon_ms < 0; clamping to 0");
      epsilon_ms_used = 0LL;
    }
    params.input.sync.epsilon_ms = static_cast<uint32_t>(epsilon_ms_used);
    params.input.topics = this->get_parameter("input.topics").as_string_array();
    params.input.frame_ids = this->get_parameter("input.frame_ids").as_string_array();
    params.output.topic = this->get_parameter("output.topic").as_string();
    params.output.frame_id = this->get_parameter("output.frame_id").as_string();

    const auto transform_type_str = this->get_parameter("transform.type").as_string();
    try {
      params.transform_cfg.type = parseTransformType(transform_type_str);
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

    printNodeParams(this->get_logger(), params);
    
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
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    sync_queue_.push(SyncedData{a, b});
  }
  queue_cv_.notify_one();
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
  if (params_.transform_cfg.type == TransformType::STATIC) {
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

    return transformed;
  } catch (const std::exception & ex) {
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

  const bool areFramesNotMatching = transformed_a->header.frame_id != params_.output.frame_id ||
      transformed_b->header.frame_id != params_.output.frame_id;
  if (areFramesNotMatching) {
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
  fused->header.stamp = this->get_clock()->now();
  // fused->header.stamp = (rclcpp::Time(transformed_a->header.stamp) >= rclcpp::Time(transformed_b->header.stamp))
  //                         ? transformed_a->header.stamp
  //                         : transformed_b->header.stamp;

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


  return fused;
}

}  // namespace fusion
