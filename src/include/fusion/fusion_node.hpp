#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_epsilon_time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include "fusion/params.hpp"
#include <memory>
#include <string>
#include <unordered_map>

#include <geometry_msgs/msg/transform_stamped.hpp>

namespace fusion
{

typedef message_filters::sync_policies::ApproximateEpsilonTime<
          sensor_msgs::msg::PointCloud2,
          sensor_msgs::msg::PointCloud2
        > FusionSyncPolicy;

class FusionNode : public rclcpp::Node
{
public:
  explicit FusionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloudA_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloudB_;
  std::shared_ptr<message_filters::Synchronizer<FusionSyncPolicy>> sync_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  FusionNodeParameters params_{};
  tf2_ros::Buffer tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unordered_map<std::string, geometry_msgs::msg::TransformStamped> static_tf_cache_;
  bool is_transmitting_{false};

  FusionNodeParameters handleParams();

  geometry_msgs::msg::TransformStamped lookupTransformToOutputFrame(
    const std::string & source_frame
  );

  sensor_msgs::msg::PointCloud2::ConstSharedPtr transformToOutputFrame(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud
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
