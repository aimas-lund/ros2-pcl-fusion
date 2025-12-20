#pragma once
#include <string>
#include <vector>
#include <cstdint>

namespace fusion
{

enum class TransformType
{
    STATIC,
    DYNAMIC
};

struct FusionNodeParameters
{
struct Input
{
    std::vector<std::string> topics;
    std::vector<std::string> frame_ids;
    struct Sync
    {
    uint32_t queue_size;
    double slop;
    } sync;
} input;

struct Output
{
    std::string topic;
    std::string frame_id;
} output;

struct Transform
{
    TransformType type;  // "static" or "dynamic"
} transform;
};

}  // namespace fusion