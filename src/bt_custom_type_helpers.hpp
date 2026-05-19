#pragma once

#include <nlohmann/json.hpp>

#include <behaviortree_cpp/json_export.h>

using json = nlohmann::json;

// Custom types
struct Pose2D
{
    Pose2D():
        x(0.0), y(0.0), yaw(0.0) {}
    Pose2D(double x, double y, double yaw):
        x(x), y(y), yaw(yaw) {}        
    double x, y, yaw;
};

BT_JSON_CONVERTER(Pose2D, pose2d)
{
  add_field("x", &pose2d.x);
  add_field("y", &pose2d.y);
  add_field("yaw", &pose2d.yaw);
}

struct Pose2DRelative
{
    Pose2DRelative():
        distance(0.0), heading(0.0) {}
    Pose2DRelative(double distance, double heading):
        distance(distance), heading(heading) {}        
    double distance, heading;
};

BT_JSON_CONVERTER(Pose2DRelative, pose2d_relative)
{
  add_field("distance", &pose2d_relative.distance);
  add_field("heading", &pose2d_relative.heading);
}

struct Position
{
    Position():
        x(0.0), y(0.0), z(0.0) {}
    Position(double x, double y, double z):
        x(x), y(y), z(z) {}
    double x, y, z;
};

BT_JSON_CONVERTER(Position, position)
{
  add_field("x", &position.x);
  add_field("y", &position.y);
  add_field("z", &position.z);
}

struct OrientationRPY
{
    OrientationRPY():
        r(0.0), p(0.0), y(0.0) {}
    OrientationRPY(double r, double p, double y):
        r(r), p(p), y(y) {}
    double r, p, y;
};

BT_JSON_CONVERTER(OrientationRPY, orientation_rpy)
{
  add_field("r", &orientation_rpy.r);
  add_field("p", &orientation_rpy.p);
  add_field("y", &orientation_rpy.y);
}

namespace BT
{
template <> inline
Pose2D convertFromString(StringView key)
{
    if (key.size() > 0 && key[0] == '{') {
        try {
            json j = json::parse(key);
            
            Pose2D output;
            output.x = j["x"];
            output.y = j["y"];
            output.yaw = j["yaw"];
		    return output;

        } catch (json::parse_error& ex) {
            RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Pose2D, str not json, at: %ld", ex.byte);
        }            
    }

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
Pose2DRelative convertFromString(StringView key)
{
    if (key.size() > 0 && key[0] == '{') {
        try {
            json j = json::parse(key);
            
            Pose2DRelative output;
            output.distance = j["distance"];
            output.heading = j["heading"];
		    return output;

        } catch (json::parse_error& ex) {
            RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Pose2DRelative, str not json, at: %ld", ex.byte);
        }            
    }

    // three real numbers separated by commas
    auto parts = BT::splitString(key, ',');
    if (parts.size() != 2)
    {
        throw BT::RuntimeError("invalid input)");
    }
    else
    {
        Pose2DRelative output;
        output.distance = convertFromString<double>(parts[0]);
        output.heading = convertFromString<double>(parts[1]);
		return output;
    }
}

inline
std::string convertToString(const Pose2DRelative &pose)
{
	std::stringstream str;
	str << pose.distance << ","
		<< pose.heading
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

inline
std::vector<Pose2D> convertPose2DListFromString(BT::StringView key, char sep, bool swap_x_y = false)
{
    std::vector<Pose2D> poses;

    // One or more groups of 3 real numbers separated by the specified one-character separator.
    // Ex using space separator:
    //   22.7,1.5,2.0 5.1,8.22,0
    auto groups = BT::splitString(key, sep);
    if (groups.size() >= 1)
    {
        for (const auto &group: groups) {
            auto pose_2d = BT::convertFromString<Pose2D>(group);
            if (swap_x_y) {
                auto tmp = pose_2d.x;
                pose_2d.x = pose_2d.y;
                pose_2d.y = tmp;
            }
            poses.push_back(pose_2d);
        }
    }
    return poses;        
}

inline
void RegisterCustomTypeHelpersJson()
{
    BT::RegisterJsonDefinition<Position>();
    BT::RegisterJsonDefinition<Pose2D>();
    BT::RegisterJsonDefinition<OrientationRPY>();
}