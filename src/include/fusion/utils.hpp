#pragma once

#include "fusion/params.hpp"

#include <algorithm>
#include <cctype>
#include <initializer_list>
#include <string_view>
#include <stdexcept>
#include <string>
#include <utility>

namespace fusion
{

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

}  // namespace fusion
