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

}  // namespace fusion
