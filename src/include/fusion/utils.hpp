#pragma once

#include "fusion/params.hpp"

#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <cctype>
#include <initializer_list>
#include <string_view>
#include <stdexcept>
#include <string>
#include <utility>

namespace fusion
{

  /**
   * Generic enum parser from string
   * @tparam Enum The enum type to parse
   * @param value The string value to parse
   * @param mapping An initializer list of string-enum pairs for mapping
   * @param parameter_name The name of the parameter (for error messages)
   * @return The parsed enum value
   * @throws std::invalid_argument if the value is not found in the mapping
   * @note Parameter strings are parsed to lowercase before comparison
   */
template <typename Enum>
inline Enum parseEnum(
  std::string_view value,
  std::initializer_list<std::pair<std::string_view, Enum>> mapping,
  std::string_view parameter_name)
{
  std::string normalized(value);
  std::transform(
    normalized.begin(),
    normalized.end(),
    normalized.begin(),
    [](unsigned char c) { return std::tolower(c); });

  for (const auto & kv : mapping) {
    if (normalized == kv.first) {
      return kv.second;
    }
  }

  throw std::invalid_argument(std::string(parameter_name) + " has an invalid value");
}

inline TransformType parseTransformType(const std::string &value)
{
  return parseEnum<TransformType>(
    value,
    {
        {"static", TransformType::STATIC}, {"dynamic", TransformType::DYNAMIC}
    }, "transform.type");
}

inline void printNodeParams(const rclcpp::Logger &logger, const FusionNodeParameters &params)
{
  const auto [slop_sec, slop_nsec] = params.input.sync.slop;
  const int64_t slop_ms = static_cast<int64_t>(slop_sec) * 1000LL +
    static_cast<int64_t>(slop_nsec) / 1000000LL;

  RCLCPP_INFO(logger, "FusionNode parameters:");
  RCLCPP_INFO(
    logger,
    "  input.topics: [%s, %s]",
    params.input.topics[0].c_str(),
    params.input.topics[1].c_str());
  RCLCPP_INFO(
    logger,
    "  input.frame_ids: [%s, %s]",
    params.input.frame_ids[0].c_str(),
    params.input.frame_ids[1].c_str());
  RCLCPP_INFO(logger, "  input.sync.queue_size: %u", params.input.sync.queue_size);
  RCLCPP_INFO(logger, "  input.sync.slop_ms: %ld", static_cast<long>(slop_ms));
  RCLCPP_INFO(logger, "  output.topic: %s", params.output.topic.c_str());
  RCLCPP_INFO(logger, "  output.frame_id: %s", params.output.frame_id.c_str());
  RCLCPP_INFO(logger, "  transform.type: %u", static_cast<uint32_t>(params.transform_cfg.type));
}

}  // namespace fusion
