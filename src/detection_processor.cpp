/*
Copyright 2021 Scott Horton

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

/*
This class subscribes to a specified topic that receives messages of type
object_detection_msgs::msg::ObjectDescArray.  For each update each
detected object is mapped to a pre-specified frame and then added/updated
to the current list of detected objects.

When the 'select' method is called, the detection best fitting the specified
selection metric is determined.  This result is compared to result of the
previous (if any) 'select' call with the best of the two then being saved.A 
call to 'getSelected' will return an object describing the selected detection
including:
- x,y,z in the pre-selected frame
- yaw angle relative to robot
- distance from robot
- token provided with the call to 'select' for which this object was selected
  (used to remember parameters for how the detection was made such as head pose)

The selection metrics supported are:
- Closest

TODO:
Cleanup methods since there is some redundancy and messiness.
Improve processing of updates to make it more efficient and better able to
  track objects between updates.
Make thread safe.
*/


#include <iostream>
#include <vector>
#include <string>
#include <map>
#include <algorithm>
#include <random>
#include <chrono>
#include <cmath>

#include "transform_helper.hpp"
#include "detection_processor.hpp"
#include "robot_status.hpp"

#define TIMER_UPDATE_PERIOD	0.5
const std::string ROBOT_BASE_LINK = "base_link";

ObjDetProc::ObjDetProc(rclcpp::Node::SharedPtr node, const std::string &name, const std::string &topic):
	node_(node), name_(name), detection_topic_(topic), min_det_count_(1), drop_det_count_(3),
	spatial_tolerance_(0.1), process_(true), selected_dist_(0.0), obj_id_(0), received_(false)
{
	detected_obj_sub_ = node_->create_subscription<object_detection_msgs::msg::ObjectDescArray>(
		detection_topic_,
		rclcpp::SystemDefaultsQoS(),
		std::bind(&ObjDetProc::DetectionCallback, this, std::placeholders::_1));
}

ObjDetProc::~ObjDetProc()
{
}

bool ObjDetProc::Configure(const std::map<std::string, double> &objects_to_det, int min_det_count, int drop_det_count,
							int det_timeout, double spatial_tolerance, const std::string &coord_frame,
							const std::string &pub_topic, const std::string &pub_frame)
{
	bool new_pub = pub_topic_ != pub_topic;

	objects_to_det_ = objects_to_det;
	min_det_count_ = min_det_count;
	drop_det_count_ = drop_det_count;
	det_timeout_ = det_timeout;
	spatial_tolerance_ = spatial_tolerance;
	coord_frame_ = coord_frame;
	pub_topic_ = pub_topic;
	pub_frame_ = pub_frame;

	if (new_pub) {
//		obj_pub_ = node_->create_publisher<object_detection_msgs::msg::ObjectDescArray>(pub_topic_, 10);
		obj_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(pub_topic_, 10);
	}

	rclcpp::Duration period = rclcpp::Duration::from_seconds(TIMER_UPDATE_PERIOD);
	timer_ = rclcpp::create_timer(node_, node_->get_clock(), period, std::bind(&ObjDetProc::DetectionTimeout, this));

	Reset();
	RCLCPP_INFO(node_->get_logger(), "Configured detection processor [%s]",
			name_.c_str());
	return true;
}

void ObjDetProc::DetectionCallback(object_detection_msgs::msg::ObjectDescArray::SharedPtr msg)
{
	auto objs = msg->objects;
	RCLCPP_INFO(node_->get_logger(), "DetectionCallback [%s], det cnt: [%lu], Num existing: [%lu]",
		name_.c_str(), objs.size(), active_detections_.size());

	received_ = true;

	if (!process_) {
		return;
	}

	RCLCPP_DEBUG(node_->get_logger(), "Got detected objects");

	// Clear the bookkeeping flag for the existing items that is
	// used to mark whether an existing object has been matched to a new one
	for (auto &ck: active_detections_) {
		ck.matched = false;
	}

	double max_dist = spatial_tolerance_*spatial_tolerance_;
	double ck_dist = 0.3;

	for (auto &o: objs) {
		// Make sure we care about this class
		auto it = objects_to_det_.find(o.name);
		if (it != objects_to_det_.end() &&
			it->second < o.confidence) {

			double x = o.x/1000.0;
			double y = o.y/1000.0;
			double z = o.z/1000.0;

			// fix - ignore spurious location 0,0,0 detections
			if (x < 0.01 && y < 0.01 && z < 0.01) {
				RCLCPP_ERROR(node_->get_logger(), "Ignoring 0,0,0 detection");
				continue;
			}

			// Convert position to odom frame to account for movement when correlating
			// subsequent detections.
			if (TransformHelper::Instance(node_).Transform(o.frame, "odom", x, y, z)) {
				o.x = x;
				o.y = y;
				o.z = z;
				o.frame = "odom";
			}

			// Also convert to the specified frame
			if (coord_frame_.length()) {
				TransformHelper::Instance(node_).Transform("odom" /*o.frame*/, coord_frame_, x, y, z);
			}

			bool found = false;
			// Try to update existing detection

#if 1
			// FIX - improve to avoid o(n^2)
			double best_dist = std::numeric_limits<double>::infinity();
			DetObj *best = nullptr;

			for (auto &ck: active_detections_) {
				//if (ck.matched) {
				//	continue;
				//}
				// Only compare dist if reasonably close
				if (abs(ck.desc.x - o.x) < ck_dist &&
					abs(ck.desc.y - o.y) < ck_dist &&
					abs(ck.desc.z - o.z) < ck_dist) {

					double dist = CalcSqrDist(ck.desc.x, ck.desc.y, ck.desc.z, o.x, o.y, o.z);
					if (dist < best_dist) {
						best_dist = dist;
						best = &ck;
						RCLCPP_INFO(node_->get_logger(), "obj [%lu] dist %f", ck.id, dist);
					}
				} else {
					RCLCPP_INFO(node_->get_logger(), "obj [%lu] exceeds spatial tolerance: cur (odom) %f,%f,%f  ck %f,%f,%f",
						ck.id, o.x, o.y, o.z, ck.desc.x, ck.desc.y, ck.desc.z);
				}
			}

			if (best && best_dist < max_dist) {
				found = true;
				if (!best->matched) {
					best->x = x;
					best->y = y;
					best->z = z;
					best->desc = o;
					best->matched = true;

					if (++best->cnt_det >= min_det_count_) {
						best->cnt_det = min_det_count_;
						// Since cnt_no_det is incremented below
						best->cnt_no_det = -1;
					}
					RCLCPP_INFO(node_->get_logger(), "using best obj [%lu], cnt_det [%d]", best->id, best->cnt_det);
				} else {
					RCLCPP_INFO(node_->get_logger(), "matched to  best obj [%lu], but already claimed", best->id);

				}
			}

#else
			for (auto &ck: active_detections_) {
				if (ck.matched) {
					continue;
				}
				if (abs(ck.x - x) < spatial_tolerance_ &&
					abs(ck.y - y) < spatial_tolerance_ &&
					abs(ck.z - z) < spatial_tolerance_) {

					if (++ck.cnt_det >= min_det_count_) {
						ck.cnt_det = min_det_count_;
						// Since cnt_no_det is incremented below
						ck.cnt_no_det = -1;
					}
					ck.x = x;
					ck.y = y;
					ck.z = z;
					found = true;
					ck.matched = true;
				} 
			}
#endif			
			// Add new detection
			if (!found) {
				DetObj d;
				d.desc = o;
				d.id = ++obj_id_;
				d.x = x;
				d.y = y;
				d.z = z;
				d.cnt_det = 1;
				// Since cnt_no_det is incremented below
				d.cnt_no_det = -1;
				d.matched = true;
				active_detections_.push_back(d);
				RCLCPP_INFO(node_->get_logger(), "New obj [%lu] at (map) %f, %f, %f, (odom) %f, %f, %f",
					 d.id, x, y, z, o.x, o.y, o.z);
			}
		}
	}

	Retire();
	Publish();

	std::ostringstream oss;
	oss << "IDs: ";
	for (auto &d: active_detections_) {
		oss << d.id << ", ";
	}
	RCLCPP_INFO(node_->get_logger(), "DetectionCallback [%s] finished, Obj: [%lu], %s",
		name_.c_str(), active_detections_.size(),oss.str().c_str());
}

// Retire objects no longer seen
bool ObjDetProc::Retire()
{
	bool any_retired = false;
	for (auto it = active_detections_.begin(); it != active_detections_.end(); ) {
		if (++it->cnt_no_det >= drop_det_count_) {
			it = active_detections_.erase(it);
			RCLCPP_INFO(node_->get_logger(), "Retiring object [%lu]", it->id);
			any_retired = true;
		} else {
			it++;
		}
	}
	RCLCPP_DEBUG(node_->get_logger(), "Retire, remaining objects [%lu]", active_detections_.size());

	return any_retired;
}

void ObjDetProc::DetectionTimeout()
{
	RCLCPP_DEBUG(node_->get_logger(), "DetectionTimeout [%s]", name_.c_str());

	if (!process_) {
		return;
	}

	bool had_received = received_;
	received_ = false;
	if (had_received) {
		return;
	}
	if (Retire()) {
		Publish();
	}
}

void ObjDetProc::Publish()
{
	if (!obj_pub_) {
		return;
	}

	//object_detection_msgs::msg::ObjectDescArray pub_list;
	visualization_msgs::msg::MarkerArray pub_list;
	int cnt = 0;

	visualization_msgs::msg::Marker marker;
	marker.header.stamp = rclcpp::Time();
	marker.header.frame_id = coord_frame_;
	marker.type = visualization_msgs::msg::Marker::SPHERE;
	marker.action = visualization_msgs::msg::Marker::ADD;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 0.06;
	marker.scale.y = 0.06;
	marker.scale.z = 0.06;
	marker.color.r = 0.0;
	marker.color.g = 0.0;
	marker.color.b = 1.0;
	marker.color.a = 1.0;
	marker.lifetime.sec = 1;
	marker.lifetime.nanosec = 0;
	marker.frame_locked = true;

	for (auto &d: active_detections_) {
		if (IsActive(d)) {
			//pub_list.objects.push_back(it->desc);
			marker.id = d.id;
			marker.pose.position.x = d.x;
			marker.pose.position.y = d.y;
			marker.pose.position.z = d.z;
			pub_list.markers.push_back(marker);
		}	
	}

	#if 0
	if (pub_frame_.length() && pub_frame_ != coord_frame_) {
		Transform(pub_list, pub_frame_);
	}
	#endif

	obj_pub_->publish(pub_list);
	RCLCPP_DEBUG(node_->get_logger(), "Published %d objects", cnt);
}

bool ObjDetProc::IsActive(DetObj &obj)
{
	return obj.cnt_det >= min_det_count_;
}

void ObjDetProc::Transform(object_detection_msgs::msg::ObjectDescArray &list, const std::string &frame)
{
	for (auto &obj: list.objects) {
		if (frame != obj.frame) {
			double x = obj.x;
			double y = obj.y;
			double z = obj.z;
			TransformHelper::Instance(node_).Transform(obj.frame, frame, x, y, z);
			obj.x = x;
			obj.y = y;
			obj.z = z;
		}
	}
}

bool ObjDetProc::Select(const std::string &obj_class, const ObjDetProc::SelectionMetric &metric, const std::string &token)
{
	DetObj closest;
	double dist;

	// Get the robot position
	RobotStatus* robot_status = RobotStatus::GetInstance();
	if (!robot_status) {
		RCLCPP_ERROR(node_->get_logger(), "Cannot get robot status for pose");
		return false;
	}
	double x, y, z, yaw;
	if (!robot_status->GetPose(x, y, z, yaw)) {
		RCLCPP_ERROR(node_->get_logger(), "Robot pose not available");
		return false;
	}

	// Transform the robot position to the same frame used for the object positions
	if (coord_frame_ != "map") {
		if (!TransformHelper::Instance(node_).Transform("map", coord_frame_, x, y, z)) {
			RCLCPP_ERROR(node_->get_logger(), "Failed to transform robot pose to frame: [%s]", coord_frame_.c_str());
			return false;
		}
		RCLCPP_ERROR(node_->get_logger(), "Transforming robot pose to frame: [%s]", coord_frame_.c_str());
	}

	if (!ObjDetProc::Closest(obj_class, x, y, z, false, closest, dist)) {
		return false;
	}
	// Already one selected?
	bool use_new = true;
	if (selected_.cnt_det > 0) {
		if (dist > selected_dist_) {
			// Keep previous selection
			use_new = false;
		}
	}
	if (use_new) {
		selected_ = closest;
		selected_dist_ = dist;
		selected_token_ = token;
	}
	RCLCPP_ERROR(node_->get_logger(), "Selected object [%lu], x,y,z: %f, %f, %f",
		selected_.id, selected_.x, selected_.y, selected_.z);
	return true;
}

void ObjDetProc::DeSelect()
{
	selected_.cnt_det = 0;
}

bool ObjDetProc::GetSelected(ObjDetProc::Detection &detection)
{
	if (selected_.cnt_det > 0) {
		return ObjDetProc::LocalObjToDetection(selected_, selected_dist_, selected_token_, detection);
	}
	return false;
}

bool ObjDetProc::LocalObjToDetection(const DetObj &obj, double dist, const std::string &token, Detection &det)
{
	det.id = obj.id;
	det.token = token;
	det.dist = sqrt(dist);
	det.pos = {obj.x, obj.y, obj.z};

	// Transform the object pos to be relative to the base of the robot and use that
	// to calc the yaw.
	double x = obj.x;
	double y = obj.y;
	double z = obj.z;
	if (!TransformHelper::Instance(node_).Transform(coord_frame_, ROBOT_BASE_LINK, x, y, z)) {
		return false;
	}
	det.yaw = std::atan2(y, x)*180.0/acos(-1.0);
	return true;
}

bool ObjDetProc::GetObjectPos(size_t id, double &x, double &y, double &z, const std::string &coord_frame)
{
	for (auto &o: active_detections_) {
		if (o.id == id) {
			x = o.desc.x;
			y = o.desc.y;
			z = o.desc.z;
			if (!TransformHelper::Instance(node_).Transform(o.desc.frame, coord_frame, x, y, z)) {
				return false;
			}
			RCLCPP_ERROR(node_->get_logger(), "GetObjectPos: id: [%lu], xyz(%s) %f, %f, %f, xyz(%s) %f, %f, %f",
				id, o.desc.frame.c_str(), o.desc.x, o.desc.y, o.desc.z, coord_frame.c_str(), x, y, z);
			return true;
		}
	}
	return false;
}

bool ObjDetProc::GetClosestTo(const ObjDetProc::Position &pos, bool use_z, ObjDetProc::Detection &detection)
{
	DetObj closest;
	double closest_dist;
	if (!Closest("", pos.x, pos.y, pos.z, use_z, closest, closest_dist)) {
		return false;
	}
	return LocalObjToDetection(closest, closest_dist, "", detection);
}

void ObjDetProc::ClearCollection()
{
	active_detections_.clear();
	received_ = false;
}

void ObjDetProc::Reset()
{
	ClearCollection();
	DeSelect();
}

void ObjDetProc::Pause()
{
	process_ = false;
}

void ObjDetProc::Resume()
{
	process_ = true;
}

inline double ObjDetProc::CalcSqrDist(double x1, double y1, double z1, double x2, double y2, double z2)
{
	return (x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2);
}

bool ObjDetProc::Closest(const std::string &obj_class, double x, double y, double z, bool use_z, DetObj &closest, double &dist)
{
	closest.cnt_det = 0;
	double closest_dist = std::numeric_limits<double>::infinity();

	bool filter_class = obj_class.length();

	for (auto &det: active_detections_) {
		if (IsActive(det) && (!filter_class || det.desc.name == obj_class)) {
			dist = CalcSqrDist(x, y, use_z? z: 0.0, det.x, det.y, use_z? det.z: 0.0);
			if (dist < closest_dist) {
				closest = det;
				closest_dist = dist;
			}
		}			
	}
	if (closest.cnt_det == 0) {
		return false;
	}
	dist = closest_dist;
	return true;
}


