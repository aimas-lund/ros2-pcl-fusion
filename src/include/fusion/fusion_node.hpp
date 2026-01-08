#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>

#if defined(FUSION_ROS_DISTRO_HUMBLE)
  #include <message_filters/sync_policies/approximate_time.h>
#else
  #include <message_filters/sync_policies/approximate_epsilon_time.h>
#endif
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include "fusion/params.hpp"
#include <memory>
#include <string>
#include <unordered_map>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>

#include <geometry_msgs/msg/transform_stamped.hpp>

namespace fusion
{

#if defined(FUSION_ROS_DISTRO_HUMBLE)
typedef message_filters::sync_policies::ApproximateTime<
          sensor_msgs::msg::PointCloud2,
          sensor_msgs::msg::PointCloud2
        > FusionSyncPolicy;
#else
typedef message_filters::sync_policies::ApproximateEpsilonTime<
          sensor_msgs::msg::PointCloud2,
          sensor_msgs::msg::PointCloud2
        > FusionSyncPolicy;
#endif

class FusionNode : public rclcpp::Node
{
public:
  explicit FusionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~FusionNode();

private:
  struct SyncedData {
    sensor_msgs::msg::PointCloud2::ConstSharedPtr a;
    sensor_msgs::msg::PointCloud2::ConstSharedPtr b;
  };

  // sync queue & corresponding primitives
  std::queue<SyncedData> sync_queue_;
  std::mutex sync_queue_mutex_;
  std::condition_variable sync_queue_cv_;

  std::thread worker_thread_;
  std::mutex transmit_mutex_;

  // ROS2 interfaces
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloudA_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloudB_;
  std::shared_ptr<message_filters::Synchronizer<FusionSyncPolicy>> sync_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

  // utils
  FusionNodeParameters params_{};
  tf2_ros::Buffer tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unordered_map<std::string, geometry_msgs::msg::TransformStamped> static_tf_cache_;

  // flags
  bool stop_worker_ = false;
  bool is_transmitting_ = false;

  // worker thread intermediate fuse buffers
  sensor_msgs::msg::PointCloud2 transformed_a_buffer_;
  sensor_msgs::msg::PointCloud2 transformed_b_buffer_;

  // methods
  void workerLoop();
  void processSyncedData(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &a,
                       const sensor_msgs::msg::PointCloud2::ConstSharedPtr &b);

  FusionNodeParameters handleParams();

  geometry_msgs::msg::TransformStamped lookupTransformToOutputFrame(
    const std::string & source_frame
  );

 const sensor_msgs::msg::PointCloud2 * transformToOutputFrame(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & in,
    sensor_msgs::msg::PointCloud2 & out
  );

  void syncCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & a,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & b
  );

  sensor_msgs::msg::PointCloud2::UniquePtr fuse(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & a,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & b
  );
};

}  // namespace fusion
