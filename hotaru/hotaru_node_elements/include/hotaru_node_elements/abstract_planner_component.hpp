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

struct HotaruPlannerState
{
	geometry_msgs::PoseStamped  current_pose;
	geometry_msgs::TwistStamped current_velocity;
};

class HotaruPlannerNode
{
private:
	// Trajectory handling
	TrajectorySlicer slicer;

protected:
	///< State definition for all planners
	std::unique_ptr<HotaruPlannerState> plannerstate;
	// ROS nodehandles
	ros::NodeHandle nh;
	ros::NodeHandle private_nh;
	// Timers
	ros::Timer planner_cycle;

	virtual void config() = 0;


public:
	HotaruPlannerNode(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh):
		nh(nh),
		private_nh(private_nh){}

	virtual bool init() = 0;
};

}

#endif /* INCLUDE_HOTARU_NODE_ELEMENTS_ABSTRACT_PLANNER_COMPONENT_HPP_ */
