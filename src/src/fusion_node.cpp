#include "fusion/fusion_node.hpp"
#include "fusion/utils.hpp"

#include <cstring>
#include <functional>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>
#include <chrono>

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

#if defined(FUSION_ROS_DISTRO_HUMBLE)
  sync_ = std::make_shared<
    message_filters::Synchronizer<FusionSyncPolicy>>(
    FusionSyncPolicy(params_.input.sync.queue_size),
    cloudA_, cloudB_);
  sync_->setMaxIntervalDuration(
    rclcpp::Duration::from_nanoseconds(params_.input.sync.epsilon_ms * 1000000ULL));
#else
  sync_ = std::make_shared<
    message_filters::Synchronizer<FusionSyncPolicy>>(
    FusionSyncPolicy(
      params_.input.sync.queue_size,
      rclcpp::Duration::from_nanoseconds(params_.input.sync.epsilon_ms * 1000000ULL)),
    cloudA_, cloudB_);
#endif

  sync_->registerCallback(
    std::bind(&FusionNode::syncCallback, this, std::placeholders::_1, std::placeholders::_2)
  );

  worker_thread_ = std::thread([this]() { this->workerLoop(); });
}

FusionNode::~FusionNode() {
  {
    std::lock_guard<std::mutex> lock(sync_queue_mutex_);
    stop_worker_ = true;
  }
  sync_queue_cv_.notify_one();
  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }
}

void FusionNode::workerLoop() {
  while (true) {
    SyncedData data;
    {
      std::unique_lock<std::mutex> lock(sync_queue_mutex_);
      sync_queue_cv_.wait(lock, [this]() { 
        return !sync_queue_.empty() || stop_worker_; 
      });
      if (stop_worker_) {
        // clear remaining data in queue if stopping
        while (!sync_queue_.empty()) {
          sync_queue_.pop();
        }
        break;
      }
      data = std::move(sync_queue_.front());
      sync_queue_.pop();
    }
    processSyncedData(data.a, data.b);
  }
}

void FusionNode::processSyncedData(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &a,
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
  const int64_t default_epsilon_ms = 100LL;
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
      "input.sync.epsilon_ms",
      rclcpp::ParameterValue(static_cast<int64_t>(default_epsilon_ms)));
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

    if (queue_size_param <= 0U) {
      params.input.sync.queue_size = default_queue_size;
      RCLCPP_WARN(
        this->get_logger(),
        "Configured input.sync.queue_size <= 0; defaulting to %u",
        default_queue_size);
    } else {
      params.input.sync.queue_size = static_cast<uint32_t>(queue_size_param);
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
    std::lock_guard<std::mutex> lock(sync_queue_mutex_);
    bool dropped_any_msg = false;
    while (sync_queue_.size() >= params_.input.sync.queue_size) {
      sync_queue_.pop();
      dropped_any_msg = true;
    }

    if (dropped_any_msg) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Sync queue full (size=%u). Dropping oldest data...",
        params_.input.sync.queue_size);
    }
    sync_queue_.push(SyncedData{a, b});
  }
  sync_queue_cv_.notify_one();
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
const sensor_msgs::msg::PointCloud2 * FusionNode::transformToOutputFrame(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & in,
  sensor_msgs::msg::PointCloud2 & out)
{
  if (!in) {
    return nullptr;
  }

  if (in->header.frame_id == params_.output.frame_id) {
    return in.get();
  }

  try {
    const auto transform = lookupTransformToOutputFrame(in->header.frame_id);

    out.fields.reserve(in->fields.size());
    out.data.reserve(in->data.size());

    tf2::doTransform(*in, out, transform);

    return &out;
  } catch (const std::exception & ex) {
    RCLCPP_WARN(this->get_logger(),
      "Failed to transform cloud from '%s' to '%s': %s",
      in->header.frame_id.c_str(),
      params_.output.frame_id.c_str(),
      ex.what());
    return nullptr;
  }
}

/**
 * Check if two PointCloud2 messages have matching layouts
 * @param a First point cloud
 * @param b Second point cloud
 * @return True if layouts match, false otherwise
 * @note Not necessary for point-wise operations, but required for fast concatenation
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
  const auto transformed_a_ptr = transformToOutputFrame(a, this->transformed_a_buffer_);
  const auto transformed_b_ptr = transformToOutputFrame(b, this->transformed_b_buffer_);

  if (!transformed_a_ptr || !transformed_b_ptr) {
    return nullptr;
  }

  // frame and layout matching might be checks that can be done once, assuming point clouds
  // always come from the same sources. Might be option for later
  const bool areFramesNotMatching = transformed_a_ptr->header.frame_id != params_.output.frame_id ||
      transformed_b_ptr->header.frame_id != params_.output.frame_id;
  if (areFramesNotMatching) {
    RCLCPP_WARN(this->get_logger(), "Transformed clouds are not in output frame.");
    return nullptr;
  }

  if (!hasMatchingPointCloud2Layout(*transformed_a_ptr, *transformed_b_ptr)) {
    RCLCPP_WARN(this->get_logger(), "PointCloud2 layouts differ. Cannot fast-concatenate");
    return nullptr;
  }

  const size_t step = transformed_a_ptr->point_step;
  const bool isPointStepInvalid = step == 0U;
  if (unlikely(isPointStepInvalid)) {
    RCLCPP_WARN(this->get_logger(), "Cannot fuse PointCloud2 with point_step of zero");
    return nullptr;
  }

  const size_t bytes_a = transformed_a_ptr->data.size();
  const size_t bytes_b = transformed_b_ptr->data.size();
  const bool isDataSizeInvalid = (bytes_a % step != 0U) || (bytes_b % step != 0U);
  if (unlikely(isDataSizeInvalid)) {
    RCLCPP_WARN(this->get_logger(), "Cannot fuse PointCloud2 with data size not divisible by point_step");
    return nullptr;
  }

  const size_t points_a = bytes_a / step;
  const size_t points_b = bytes_b / step;
  const bool isPointCountOverflowingSize = points_a > static_cast<size_t>(std::numeric_limits<uint32_t>::max()) ||
      points_b > static_cast<size_t>(std::numeric_limits<uint32_t>::max()) ||
      (points_a + points_b) > static_cast<size_t>(std::numeric_limits<uint32_t>::max());
  if (unlikely(isPointCountOverflowingSize)) {
    RCLCPP_WARN(this->get_logger(), "Cannot fuse as total point count exceeds uint32_t max");
    return nullptr;
  }

  const bool isDataSizeOverflowingSize = bytes_a > std::numeric_limits<size_t>::max() - bytes_b;
  if (unlikely(isDataSizeOverflowingSize)) {
    RCLCPP_WARN(this->get_logger(), "Cannot fuse as fused data size overflows size_t");
    return nullptr;
  }

  const size_t fused_points = points_a + points_b;
  const bool isFusedPointCountOverflowingSize = fused_points != 0U && step > (std::numeric_limits<size_t>::max() / fused_points);
  if (unlikely(isFusedPointCountOverflowingSize)) {
    RCLCPP_WARN(this->get_logger(), "Cannot fuse as fused row_step overflows size_t");
    return nullptr;
  }

  const size_t fused_row_step = fused_points * step;
  const bool isFusedRowStepOverflowingUint32 = fused_row_step > static_cast<size_t>(std::numeric_limits<uint32_t>::max());
  if (unlikely(isFusedRowStepOverflowingUint32)) {
    RCLCPP_WARN(this->get_logger(), "Cannot fuse as fused row_step exceeds uint32_t max");
    return nullptr;
  }

  const size_t fused_bytes = bytes_a + bytes_b;
  if (unlikely(fused_bytes != fused_row_step)) {
    RCLCPP_WARN(this->get_logger(), "Cannot fuse as fused data size does not match fused row_step");
    return nullptr;
  }

  auto fused = std::make_unique<sensor_msgs::msg::PointCloud2>();

  fused->header.frame_id = params_.output.frame_id;
  fused->header.stamp = this->get_clock()->now();
  // fused->header.stamp = (rclcpp::Time(transformed_a->header.stamp) >= rclcpp::Time(transformed_b->header.stamp))
  //                         ? transformed_a->header.stamp
  //                         : transformed_b->header.stamp;

  fused->fields = transformed_a_ptr->fields;
  fused->is_bigendian = transformed_a_ptr->is_bigendian;
  fused->point_step = transformed_a_ptr->point_step;
  fused->is_dense = transformed_a_ptr->is_dense && transformed_b_ptr->is_dense;

  fused->height = 1U;
  fused->width = static_cast<uint32_t>(fused_points);
  fused->row_step = static_cast<uint32_t>(fused_row_step);

  fused->data.resize(fused_bytes);
  if (bytes_a != 0U) {
    std::memcpy(fused->data.data(), transformed_a_ptr->data.data(), bytes_a);
  }
  if (bytes_b != 0U) {
    std::memcpy(fused->data.data() + bytes_a, transformed_b_ptr->data.data(), bytes_b);
  }

  return fused;
}

}  // namespace fusion
