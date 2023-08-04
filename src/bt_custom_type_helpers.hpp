#pragma once

#include "behaviortree_cpp_v3/action_node.h"

// FIX - add reference frame and timestamp to these

// Custom types
struct Pose2D
{
    double x, y;
    double yaw;
};

struct Position
{
    double x, y, z;
};

struct OrientationRPY
{
    double r, p, y;
};

namespace BT
{
template <> inline
Pose2D convertFromString(StringView key)
{
    // three real numbers separated by commas
    auto parts = BT::splitString(key, ',');
    if (parts.size() != 3)
    {
        throw BT::RuntimeError("invalid input)");
    }
    else
    {
        Pose2D output;
        output.x = convertFromString<double>(parts[0]);
        output.y = convertFromString<double>(parts[1]);
		output.yaw = convertFromString<double>(parts[2]);
		return output;
    }
}

template <> inline
Position convertFromString(StringView key)
{
    // three real numbers separated by commas
    auto parts = BT::splitString(key, ',');
    if (parts.size() != 3)
    {
        throw BT::RuntimeError("invalid input)");
    }
    else
    {
        Position output;
        output.x = convertFromString<double>(parts[0]);
        output.y = convertFromString<double>(parts[1]);
		output.z = convertFromString<double>(parts[2]);
		return output;
    }
}

template <> inline
OrientationRPY convertFromString(StringView key)
{
    // three real numbers separated by commas
    auto parts = BT::splitString(key, ',');
    if (parts.size() != 3)
    {
        throw BT::RuntimeError("invalid input)");
    }
    else
    {
        OrientationRPY output;
        output.r = convertFromString<double>(parts[0]);
        output.p = convertFromString<double>(parts[1]);
		output.y = convertFromString<double>(parts[2]);
		return output;
    }
}

} // end namespace BT

