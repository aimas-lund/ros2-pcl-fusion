#pragma once
#include <string>
#include <tuple>
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
        struct Sync
        {
            uint32_t queue_size;
            std::tuple<int32_t, uint32_t> slop;

            constexpr Sync() : queue_size(0U), slop{0, 0} {}
        };

        std::vector<std::string> topics;
        std::vector<std::string> frame_ids;
        Sync sync;
    } input;

    struct Output
    {
        std::string topic;
        std::string frame_id;
    } output;

    struct TransformConfig
    {
        TransformType type;  // "static" or "dynamic"

        constexpr TransformConfig() : type(TransformType::STATIC) {}
    } transform_cfg;
};

}  // namespace fusion