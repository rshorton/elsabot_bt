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

#ifndef _DETECTION_PROCESSOR_HPP_
#define _DETECTION_PROCESSOR_HPP_

#include <vector>
#include <string>
#include <memory>
#include <map>
#include <thread>
#include <mutex>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "object_detection_msgs/msg/object_desc_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

class ObjDetProc
{
public:
	struct Position {
		double x;
		double y;
		double z;
	};

	struct Detection {
		size_t id;
		Position pos;
		double yaw;
		double dist;
		std::string token;
	};

	enum SelectionMetric { closest };

	struct DetObj {

		object_detection_msgs::msg::ObjectDesc desc;
		size_t id;
		int cnt_det;
		int cnt_no_det;
		double x;
		double y;
		double z;
		bool updated;
		bool matched;
	};

	ObjDetProc(rclcpp::Node::SharedPtr node, const std::string &name, const std::string &topic);
	~ObjDetProc();

	// confidence per object class to be detected
	// add_det_count - minimum consecutive detections to consider detected
	// drop_det_count - number of consecutive loss of detections to consider no longer detected
	// det_timeout - number of milliseconds of no updates to consider objects no longer present
	// coord_frame - if non-empty, coordinate frame to map object before performing spatial tolerance testing
	// pub_topic - if non-empty, then publish currently valid detections to the specified topic
	// pub_frame - if non-empty, then transform coords to specified frame before publishing
	bool Configure(const std::map<std::string, double> &objects_to_det, int min_det_count, int drop_det_count,
		int det_timeout, const std::string &pub_topic);

	// obj_class - if non-empty, then limit to specified object class
	// metric - selection metric
	// token - token to assign to selection
	// count - returns the total number of objects detected
	bool Select(const std::string &obj_class, const ObjDetProc::SelectionMetric &metric, const std::string &token, int &count);

	void DeSelect();

	// Clear the current collection of detections (but not the currently selected detection)
	void ClearCollection();
	void Reset();

	// Pause processing updates
	void Pause();

	// Clear everything remembered (current collection and selection)	
	void Resume();

	// Return the object that is currently selected at the best selection from a previous 'Select' call
	bool GetSelected(ObjDetProc::Detection &detection);
	// Return the object closes to the specified position
	bool GetClosestTo(const ObjDetProc::Position &pos, bool use_z, ObjDetProc::Detection &detection);

	// Return the position of the specified object relative to the specified frame
	bool GetObjectPos(size_t id, double &x, double &y, double &z, const std::string &coord_frame);

private:
	void DetectionCallback(object_detection_msgs::msg::ObjectDescArray::SharedPtr msg);

	bool IsActive(DetObj &obj);

	void Publish();
	bool Retire();

	inline double CalcSqrDist(double x1, double y1, double z1, double x2, double y2, double z2);
	bool Closest(const std::string &obj_class, double x, double y, double z, bool use_z, DetObj &closest, double &dist);

	bool LocalObjToDetection(const DetObj &obj, double dist, const std::string &token, Detection &det);

	void DetectionTimeout();

private:
	rclcpp::Node::SharedPtr node_;
	std::string name_;
	std::string detection_topic_;
	std::map<std::string, double> objects_to_det_;
	int min_det_count_;
	int drop_det_count_;
	int det_timeout_;
	std::string coord_frame_;

	bool process_;

	DetObj selected_;
	double selected_dist_;
	std::string selected_token_;

	size_t obj_id_;
	bool received_;

    rclcpp::Subscription<object_detection_msgs::msg::ObjectDescArray>::SharedPtr detected_obj_sub_;
	//rclcpp::Publisher<object_detection_msgs::msg::ObjectDescArray>::SharedPtr obj_pub_;
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obj_pub_;

    object_detection_msgs::msg::ObjectDescArray objArray_;
    bool detected_;
    //std::chrono::time_point<std::chrono::high_resolution_clock> last_time_;

	std::vector<struct DetObj> active_detections_;

	rclcpp::TimerBase::SharedPtr timer_;
};

#endif //_DETECTION_PROCESSOR_HPP_
