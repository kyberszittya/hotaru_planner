/*
 * abstract_planner_component.hpp
 *
 *  Created on: Apr 1, 2020
 *      Author: kyberszittya
 */

#ifndef INCLUDE_HOTARU_NODE_ELEMENTS_ABSTRACT_PLANNER_COMPONENT_HPP_
#define INCLUDE_HOTARU_NODE_ELEMENTS_ABSTRACT_PLANNER_COMPONENT_HPP_

#include <hotaru_node_elements/trajectory_slicer.hpp>

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

namespace hotaru
{


template<class WaypointMsg> class HotaruPlannerNode
{
private:

protected:
	// Trajectory handling
	TrajectorySlicer slicer;
	TrajectoryMerger merger;
	//
	std::vector<WaypointMsg> waypoint_original;
	std::vector<WaypointMsg> replanned_trajectory;




public:
	HotaruPlannerNode(){}

	virtual void config() = 0;

	virtual bool plancycle() = 0;
};


class PlannerTransformState
{
protected:
	const std::string base_frame;
	const std::string global_frame;
public:
	PlannerTransformState(const std::string base_frame, const std::string global_frame):
		base_frame(base_frame),
		global_frame(global_frame){}

	const std::string getBaseFrame() { return base_frame; }
};

}

#endif /* INCLUDE_HOTARU_NODE_ELEMENTS_ABSTRACT_PLANNER_COMPONENT_HPP_ */
