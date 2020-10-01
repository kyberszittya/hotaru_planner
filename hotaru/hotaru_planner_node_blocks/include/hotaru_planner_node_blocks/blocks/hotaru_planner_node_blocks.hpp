/*
 * hotaru_planner_node_blocks.hpp
 *
 *  Created on: Sep 9, 2020
 *      Author: kyberszittya
 */

#ifndef INCLUDE_HOTARU_PLANNER_NODE_BLOCKS_BLOCKS_HOTARU_PLANNER_NODE_BLOCKS_HPP_
#define INCLUDE_HOTARU_PLANNER_NODE_BLOCKS_BLOCKS_HOTARU_PLANNER_NODE_BLOCKS_HPP_

#include <memory>

#include <ros/ros.h>

#include <hotaru_planner_msgs/TrajectoryArray.h>
#include <geometry_msgs/PoseStamped.h>

#include <Eigen/Dense>

namespace hotaru
{

namespace blocks
{

class EventReceptor
{

};

class PlannerAlgorithm
{
protected:
	std::shared_ptr<ros::NodeHandle> nh;
	ros::Timer planner_timer;
	ros::Publisher pub_trajectory;
	/// State
	geometry_msgs::PoseStamped current_state;
	geometry_msgs::PoseStamped goal_state;
	hotaru_planner_msgs::Trajectory resultant_trajectory;
public:
	PlannerAlgorithm(std::shared_ptr<ros::NodeHandle> nh): nh(nh)
	{}

	virtual ~PlannerAlgorithm() {}
	virtual bool calculateTrajectory(const geometry_msgs::PoseStamped& current_state,
			const geometry_msgs::PoseStamped& goal_state) = 0;


	hotaru_planner_msgs::Trajectory getResultantTrajectory()
	{
		return resultant_trajectory;
	}

	bool plan(const geometry_msgs::PoseStamped& current_state,
			geometry_msgs::PoseStamped& goal_state)
	{
		return calculateTrajectory(current_state, goal_state);
	}

	void init_ros_interface_timer(const double planner_timer_period)
	{
		resultant_trajectory.header.frame_id = "map";
		pub_trajectory = nh->advertise<hotaru_planner_msgs::Trajectory>("/refined_trajectory", 1);
		planner_timer = nh->createTimer(ros::Duration(planner_timer_period), &PlannerAlgorithm::cbTimer, this);

	}

	void cbTimer(const ros::TimerEvent& et)
	{
		if (plan(current_state, goal_state))
		{
			resultant_trajectory.header.stamp = ros::Time::now();
			pub_trajectory.publish(resultant_trajectory);
		}
	}
};

class TrajectorySlicer
{

};

class TrajectoryMerger
{

};

} // namespace blocks



} // namespace hotaru


#endif /* INCLUDE_HOTARU_PLANNER_NODE_BLOCKS_BLOCKS_HOTARU_PLANNER_NODE_BLOCKS_HPP_ */
