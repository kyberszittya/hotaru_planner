/*
 * main.cpp
 *
 *  Created on: Jan 30, 2020
 *      Author: kyberszittya
 */

#include <ros/ros.h>

#include <grid_map_octomap/GridMapOctomapConverter.hpp>
#include <octomap/octomap.h>

#include <memory>
#include <thread>
#include <future>

#include <hotaru_common/state_machine/state_machine.hpp>

#include <hotaru_common/planner_components/common_building_blocks.hpp>
#include <hotaru_common/planner_components/abstract_planner.hpp>

namespace hotaru
{

struct GlobalPlannerParamters
{
	bool debug;
};

class GlobalPlanner: public Abstract_RosGlobalPlanner
{
private:
	ros::Subscriber sub_grid_map;
	ros::Subscriber sub_current_pose;
	ros::Subscriber sub_current_velocity;
	ros::Subscriber sub_goal;
	// Viz
	ros::Publisher pub_visualization_current_grid_pose;
	ros::Publisher pub_visualization_current_grid_goal;
	// Parameters
	std::unique_ptr<GlobalPlannerParamters> params;
	// Topic sync
	// TODO: integrate it
	//SyncStateMachine sync_state_machine;
	// Thread
	std::thread t_global_planner;
public:
	GlobalPlanner(ros::NodeHandle& nh): Abstract_RosGlobalPlanner(nh){}

	bool initParameters()
	{
		params = std::unique_ptr<GlobalPlannerParamters>(new GlobalPlannerParamters());
		if (params == nullptr){	return false; }
		params->debug = true;
		return true;
	}

	virtual bool initNode() override
	{
		if (!initParameters())
		{
			return false;
		}
		// ROS
		sub_grid_map = nh.subscribe("grid_map", 10, &GlobalPlanner::subGridMap, this);
		return true;
	}

	void global_thread_worker()
	{

	}

	void subCurrentPose(const geometry_msgs::PoseStamped& msg)
	{
		kinematic_state->pose = msg;
	}

	void subGoal(const geometry_msgs::PoseStamped& msg)
	{
		kinematic_state->goal = msg;
	}

	void subCurrentVelocity(const geometry_msgs::TwistStamped& msg)
	{
		kinematic_state->twist = msg;
	}

	void subGridMap(const grid_map_msgs::GridMap::ConstPtr& msg)
	{
		grid_map::GridMapRosConverter::fromMessage(*msg, perception_state->global_map);
		grid_map::Position pos_current(
						kinematic_state->pose.pose.position.x,
						kinematic_state->pose.pose.position.y);
		if (perception_state->global_map.isInside(pos_current))
		{
			grid_map::Index index_current_pose;
			perception_state->global_map.getIndex(pos_current, index_current_pose);
		}
		grid_map::Position pos_goal(
				kinematic_state->goal.pose.position.x,
				kinematic_state->goal.pose.position.y);
		if (perception_state->global_map.isInside(pos_goal))
		{
			grid_map::Index index_goal;
			perception_state->global_map.getIndex(pos_goal, index_goal);
			grid_map::Position rel_pose;
			grid_map::Position next_pose;
			perception_state->global_map.getPosition(index_goal, rel_pose);
			grid_map::Index next_goal = index_goal;
			next_goal[0] += 1;
			perception_state->global_map.getPosition(index_goal, rel_pose);
			perception_state->global_map.getPosition(next_goal, next_pose);
			std::cout << rel_pose.transpose() <<'\n' << next_pose.transpose() <<'\n';
		}
	}

	void stop()
	{
		t_global_planner.join();
	}
};

class GridMapInterface
{
private:
public:

};

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "hotaru_global_planner");
	ros::NodeHandle nh;
	hotaru::GlobalPlanner global_planner(nh);
	if (global_planner.init())
	{

		ros::spin();
		return 0;
	}
	return -1;

}
