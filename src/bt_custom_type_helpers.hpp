#pragma once

#include "behaviortree_cpp_v3/action_node.h"

// Custom types
struct Pose2D
{
    Pose2D():
        x(0.0), y(0.0), yaw(0.0) {}
    Pose2D(double x, double y, double yaw):
        x(x), y(y), yaw(yaw) {}        
    double x, y, yaw;
};

struct Position
{
    Position():
        x(0.0), y(0.0), z(0.0) {}
    Position(double x, double y, double z):
        x(x), y(y), z(z) {}
    double x, y, z;
};

struct OrientationRPY
{
    OrientationRPY():
        r(0.0), p(0.0), y(0.0) {}
    OrientationRPY(double r, double p, double y):
        r(r), p(p), y(y) {}
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

inline
std::string convertToString(const Pose2D &pose)
{
	std::stringstream str;
	str << pose.x << ","
		<< pose.y << ","
        << pose.yaw
		<< std::endl;
    return str.str();
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

inline
std::string convertToString(const Position &pos)
{
	std::stringstream str;
	str << pos.x << ","
		<< pos.y << ","
        << pos.z
		<< std::endl;
    return str.str();
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

inline
std::string convertToString(const OrientationRPY &pose)
{
	std::stringstream str;
	str << pose.r << ","
		<< pose.p << ","
        << pose.y
		<< std::endl;
    return str.str();
}

} // end namespace BT

