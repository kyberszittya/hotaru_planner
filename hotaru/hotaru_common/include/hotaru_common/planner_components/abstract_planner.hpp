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
#include <std_msgs/Int32.h>
#include <tf/transform_datatypes.h>

#include "common_building_blocks.hpp"

namespace hotaru
{

class Abstract_RosPlanner;

class LanePropertiesGetter
{
private:
	ros::NodeHandle nh;
	int current_idx;

protected:
	ros::Subscriber sub_current_wp_idx;
public:
	LanePropertiesGetter(ros::NodeHandle& nh): nh(nh), current_idx(-1)
	{

	}

	void init()
	{
		sub_current_wp_idx = nh.subscribe("closest_waypoint", 10, &LanePropertiesGetter::subClosestWaypoint, this);
	}

	void subClosestWaypoint(const std_msgs::Int32::ConstPtr& msg)
	{
		current_idx = msg->data;
	}

	friend Abstract_RosPlanner;
};

class Abstract_RosPlanner
{
protected:
	ros::Subscriber sub_base_waypoints;
	ros::Subscriber sub_current_pose;
	ros::Subscriber sub_current_velocity;
	// Subscribe to environment map
	ros::Subscriber sub_grid_map;
	// States
	//std::unique_ptr<hotaru::PlannerKinematicState> kinematic_state;
	//std::unique_ptr<hotaru::PlannerPerceptionState> perception_state;

	/*
	virtual void initRos()
	{
		sub_current_pose = nh.subscribe("current_pose", 10,
				&Abstract_RosPlanner::subCurrentPose, this);
		sub_current_velocity = nh.subscribe("current_velocity", 10,
				&Abstract_RosPlanner::subCurrentVelocity, this);

	}
	*/

public:
	virtual ~Abstract_RosPlanner() = 0;

	//Abstract_RosPlanner(ros::NodeHandle& nh): nh(nh){}

	//virtual bool initNode() = 0;

	/*
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
	*/

	/*
	void subCurrentPose(const geometry_msgs::PoseStamped::ConstPtr& msg)
	{
		kinematic_state->pose = *msg;
		tf::poseStampedMsgToTF(*msg, kinematic_state->tf_pose);
		kinematic_state->tf_proj_pose[0] = msg->pose.position.x;
		kinematic_state->tf_proj_pose[1] = msg->pose.position.y;
	}
	*/

	/*
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
	*/
	/*
	void subCurrentVelocity(const geometry_msgs::TwistStamped::ConstPtr& msg)
	{
		kinematic_state->twist = *msg;
	}
	*/


};

class Abstract_RosLocalPlanner: public Abstract_RosPlanner
{
protected:
	/*
	ros::Subscriber sub_base_waypoints;
	ros::Publisher pub_final_waypoints;
	*/
	std::vector<geometry_msgs::PoseStamped> starting_plan_points;
	std::vector<geometry_msgs::TwistStamped> original_velocity_profile;
	autoware_msgs::Lane final_waypoints;
	unsigned int number_of_trajectory_points;

	/*
	virtual void initRos()
	{
		Abstract_RosPlanner::initRos();

		sub_base_waypoints = nh.subscribe("base_waypoints", 10,
						&Abstract_RosLocalPlanner::subBaseWaypoints, this);
		pub_final_waypoints = nh.advertise<autoware_msgs::Lane>("final_waypoints", 1);

	}
	*/

	void reconstructStartingPlanPoints(const autoware_msgs::Lane& msg,
			geometry_msgs::PoseStamped pose)
	{
		starting_plan_points.clear();
		original_velocity_profile.clear();
		if (msg.waypoints.size() >= 2)
		{
			number_of_trajectory_points = 1;
			starting_plan_points.push_back(std::move(pose));
			for (int i = 1; i < msg.waypoints.size(); i++)
			{
				starting_plan_points.push_back(msg.waypoints[i].pose);
				original_velocity_profile.push_back(msg.waypoints[i].twist);
				number_of_trajectory_points++;
			}

		}
	}
public:
	//Abstract_RosLocalPlanner(ros::NodeHandle& nh): Abstract_RosPlanner(nh), number_of_trajectory_points(0){}
	Abstract_RosLocalPlanner(): number_of_trajectory_points(0){}

	virtual void executePlannerMethods() = 0;

	/*
	void subBaseWaypoints(const autoware_msgs::Lane::ConstPtr& msg)
	{
		executePlannerMethods();
		reconstructStartingPlanPoints(msg);
	}
	*/

	virtual void localPlanCycle() = 0;

};



class Abstract_RosGlobalPlanner: public Abstract_RosPlanner
{
protected:
	ros::Subscriber sub_current_goal;

	/*
	virtual void initRos()
	{
		Abstract_RosPlanner::initRos();
		sub_current_goal = nh.subscribe("current_goal", 10,
						&Abstract_RosGlobalPlanner::subGoal, this);
	}
	*/
public:
	Abstract_RosGlobalPlanner(): Abstract_RosPlanner(){}


	/*
	void subGoal(const geometry_msgs::PoseStamped::ConstPtr& msg)
	{
		kinematic_state->goal = *msg;
		tf::poseStampedMsgToTF(kinematic_state->goal, kinematic_state->tf_goal);
		std::cout << *msg << '\n';
	}
	*/
};

}

#endif /* INCLUDE_HOTARU_COMMON_PLANNER_COMPONENTS_ABSTRACT_PLANNER_HPP_ */
