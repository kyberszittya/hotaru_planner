/*
 * abstract_planner.hpp
 *
 *  Created on: Feb 10, 2020
 *      Author: kyberszittya
 */

#ifndef INCLUDE_HOTARU_COMMON_PLANNER_COMPONENTS_ABSTRACT_PLANNER_HPP_
#define INCLUDE_HOTARU_COMMON_PLANNER_COMPONENTS_ABSTRACT_PLANNER_HPP_

#include <ros/ros.h>
#include <autoware_msgs/Lane.h>
#include <tf/transform_datatypes.h>

#include "common_building_blocks.hpp"

namespace hotaru
{

class Abstract_RosPlanner
{
protected:
	ros::NodeHandle nh;
	ros::Subscriber sub_base_waypoints;
	ros::Subscriber sub_current_pose;
	ros::Subscriber sub_current_velocity;
	// Subscribe to environment map
	ros::Subscriber sub_grid_map;
	// States
	std::unique_ptr<hotaru::PlannerKinematicState> kinematic_state;
	std::unique_ptr<hotaru::PlannerPerceptionState> perception_state;

	virtual void initRos()
	{
		sub_current_pose = nh.subscribe("current_pose", 10,
				&Abstract_RosPlanner::subCurrentPose, this);
		sub_current_velocity = nh.subscribe("current_velocity", 10,
				&Abstract_RosPlanner::subCurrentVelocity, this);

	}

public:
	Abstract_RosPlanner(ros::NodeHandle& nh): nh(nh){}

	virtual bool initNode() = 0;

	virtual bool init()
	{
		if (!allocateState())
		{
			ROS_FATAL("Unable to allocate state");
			return false;
		}
		initRos();
		if (!initNode())
		{
			ROS_FATAL("Unable to initialize instance node");
			return false;
		}
		return true;
	}

	void subCurrentPose(const geometry_msgs::PoseStamped::ConstPtr& msg)
	{
		kinematic_state->pose = *msg;
		tf::poseStampedMsgToTF(*msg, kinematic_state->tf_pose);
		kinematic_state->tf_proj_pose[0] = msg->pose.position.x;
		kinematic_state->tf_proj_pose[1] = msg->pose.position.y;
	}

	bool allocateState()
	{
		kinematic_state = std::unique_ptr<PlannerKinematicState>(new PlannerKinematicState());
		if (kinematic_state==nullptr)
		{
			return false;
		}
		perception_state = std::unique_ptr<PlannerPerceptionState>(new PlannerPerceptionState());
		if (perception_state==nullptr)
		{
			return false;
		}
		return true;
	}

	void subCurrentVelocity(const geometry_msgs::TwistStamped::ConstPtr& msg)
	{
		kinematic_state->twist = *msg;
	}


};

class Abstract_RosLocalPlanner: public Abstract_RosPlanner
{
protected:
	ros::Subscriber sub_base_waypoints;
	ros::Publisher pub_final_waypoints;
	std::vector<geometry_msgs::PoseStamped> starting_plan_points;
	autoware_msgs::Lane final_waypoints;
	virtual void initRos()
	{
		Abstract_RosPlanner::initRos();
		sub_base_waypoints = nh.subscribe("base_waypoints", 10,
						&Abstract_RosLocalPlanner::subBaseWaypoints, this);
		pub_final_waypoints = nh.advertise<autoware_msgs::Lane>("final_waypoints", 1);
	}

	void reconstructStartingPlanPoints(const autoware_msgs::Lane::ConstPtr& msg)
	{
		starting_plan_points.clear();
		if (msg->waypoints.size() >= 2)
		{
			starting_plan_points.push_back(kinematic_state->pose);
			for (int i = 1; i < msg->waypoints.size(); i++)
			{
				starting_plan_points.push_back(msg->waypoints[i].pose);
			}
		}
	}
public:
	Abstract_RosLocalPlanner(ros::NodeHandle& nh): Abstract_RosPlanner(nh){}



	void subBaseWaypoints(const autoware_msgs::Lane::ConstPtr& msg)
	{
		reconstructStartingPlanPoints(msg);
	}

	virtual void localPlanCycle() = 0;

};



class Abstract_RosGlobalPlanner: public Abstract_RosPlanner
{
protected:
	ros::Subscriber sub_current_goal;

	virtual void initRos()
	{
		Abstract_RosPlanner::initRos();
		sub_current_goal = nh.subscribe("current_goal", 10,
						&Abstract_RosGlobalPlanner::subGoal, this);
	}

public:
	Abstract_RosGlobalPlanner(ros::NodeHandle& nh): Abstract_RosPlanner(nh){}


	void subGoal(const geometry_msgs::PoseStamped::ConstPtr& msg)
	{
		kinematic_state->goal = *msg;
		tf::poseStampedMsgToTF(kinematic_state->goal, kinematic_state->tf_goal);
		std::cout << *msg << '\n';
	}
};

}

#endif /* INCLUDE_HOTARU_COMMON_PLANNER_COMPONENTS_ABSTRACT_PLANNER_HPP_ */
