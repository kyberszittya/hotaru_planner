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
#include <memory>

#include <rei_statemachine_library/abstract_signal_definitions.hpp>
#include <rei_statemachine_library/abstract_statemachine_definitions.hpp>
#include <rei_statemachine_library/portsync_state_machine/sync_state_machine.hpp>
#include "common_building_blocks.hpp"
#include <rei_monitoring_msgs/ReiStateMachineTransitionSignal.h>
#include <hotaru_common/state_machine/trajectory_statemachine.hpp>
#include <hotaru_common/trajectory_utilities/trajectory_utilities.hpp>


#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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


public:
	virtual ~Abstract_RosPlanner() = 0;




};


class SyncGuardLocalPlanner: public hotaru::Interface_GuardLocalPlanner
{
protected:
	std::shared_ptr<rei::SyncStateMachine> sync_sm;

	geometry_msgs::Point lookahead_point;
	geometry_msgs::Point current_point;
	int lookahead_index;
	int current_lookahead_index;
	int minimal_velocity_distance_index;
public:
	SyncGuardLocalPlanner(std::shared_ptr<rei::SyncStateMachine> sync_sm):
		sync_sm(sync_sm),
		lookahead_index(0),
		current_lookahead_index(0),
		minimal_velocity_distance_index(0)
	{
	}

	virtual bool guard_Relay2ReplanningState() override
	{
		return sync_sm->isStarted();
	}

	virtual bool guard_Replanning2RelayState() override
	{
		return sync_sm->isStarted() &&
			(current_lookahead_index > lookahead_index
				|| minimal_velocity_distance_index > current_lookahead_index);

	}
	virtual bool guard_Relay2Waiting() override
	{
		return sync_sm->isStarted();
	}
	virtual bool guard_Waiting2Relay() override
	{
		return sync_sm->isStarted();
	}
	void setLookaheadPoint(geometry_msgs::Point msg,
			int speed_minimal_index,
			int lookahead_index,
			int trajectory_length)
	{
		minimal_velocity_distance_index = speed_minimal_index;
		lookahead_point = msg;
		this->lookahead_index = std::min(lookahead_index, trajectory_length);
	}

	void setCurrentLookaheadIndex(int index)
	{
		current_lookahead_index = index;
	}

	void setCurrentPoint(geometry_msgs::Point msg)
	{
		current_point = msg;
	}
};

const std::string translateTrajectorySignalToName(
		std::shared_ptr<rei::AbstractSignalInterface> _sig)
{
	switch(_sig->getId())
	{
		case 0x20: {
			return "SignalReplanRequest";
		}
		case 0x21: {
			return "SignalNoObstacleDetected";
		}
		case 0x22: {
			return "SignalLastWaypointReached";
		}
		case 0x23: {
			return "SignalNewGlobalPlan";
		}
		default: {
			return "INVALID";
		}
	}
}


class RosReplannerGraphNotifier: public rei::Interface_CommunicationGraphNotifier
{
private:
	std::string node_name;
	std::shared_ptr<ros::NodeHandle> nh;
	rei_monitoring_msgs::ReiStateMachineTransitionSignal msg_sig;
	ros::Publisher pub_sig_id;
public:
	RosReplannerGraphNotifier(std::string node_name,
			std::shared_ptr<ros::NodeHandle> nh):
				node_name(node_name), nh(nh) {}

	void initialize()
	{
		ROS_INFO_STREAM("Initializing ROS communication notifier: "
				<< node_name+"/sync_state_machine/current_state");
		pub_sig_id = nh->advertise<rei_monitoring_msgs::ReiStateMachineTransitionSignal>(
				node_name+"/replanner/current_state", 10);
	}


	virtual void notifyCommunicationGraph(std::shared_ptr<rei::AbstractSignalInterface> sig)
	{
		msg_sig.header.stamp = ros::Time::now();
		msg_sig.signal_name = translateTrajectorySignalToName(sig);
		msg_sig.sig_id = sig->getId();
		pub_sig_id.publish(msg_sig);
	}
};



class Abstract_RosLocalPlanner: public Abstract_RosPlanner
{
protected:

	std::vector<geometry_msgs::PoseStamped> starting_plan_points;
	std::vector<geometry_msgs::TwistStamped> original_velocity_profile;
	autoware_msgs::Lane final_waypoints;
	unsigned int number_of_trajectory_points;
	// ROS communication graph as notifier of replanner state
	std::unique_ptr<hotaru::LocalPlannerStateMachine> planner_state_machine;
	std::shared_ptr<hotaru::SyncGuardLocalPlanner> guard_local_planner;
	std::shared_ptr<RosReplannerGraphNotifier> comm_repl_notif;

	tf2_ros::Buffer tf_buffer;
	std::unique_ptr<tf2_ros::TransformListener> tf_listener;
	geometry_msgs::TransformStamped transform_current_pose;
	geometry_msgs::TransformStamped inv_transform_current_pose;
	hotaru::TrajectorySlicer trajectory_slicer;


	void syncTfPose()
	{
		try
		{
			transform_current_pose = tf_buffer.lookupTransform(
				"base_link", "map", ros::Time(0)
			);
			inv_transform_current_pose = tf_buffer.lookupTransform(
					"map", "base_link",  ros::Time(0)
			);

		}catch(tf2::LookupException& le){
			std::cerr << le.what() << '\n';
		}catch(tf2::ExtrapolationException& ee)
		{
			std::cerr << ee.what() << '\n';
		}catch(tf2::ConnectivityException& ce)
		{
			std::cerr << ce.what() << '\n';
		}
	}

	bool initLocalPlannerStateMachine(std::shared_ptr<ros::NodeHandle> nh,
			std::shared_ptr<rei::SyncStateMachine> sync_sm)
	{
		// Initialize planner state machine
		comm_repl_notif = std::shared_ptr<RosReplannerGraphNotifier>(
			new RosReplannerGraphNotifier("local_planner_state", nh)
		);
		if (comm_repl_notif == nullptr)
		{
			return false;
		}
		comm_repl_notif->initialize();
		guard_local_planner = std::unique_ptr<hotaru::SyncGuardLocalPlanner>(
				new hotaru::SyncGuardLocalPlanner(sync_sm));
		if (guard_local_planner == nullptr)
		{
			return false;
		}
		planner_state_machine = std::unique_ptr<hotaru::LocalPlannerStateMachine>(
				new hotaru::LocalPlannerStateMachine(comm_repl_notif,
						guard_local_planner));
		if (planner_state_machine == nullptr)
		{
			return false;
		}
		tf_listener = std::unique_ptr<tf2_ros::TransformListener>(
			new tf2_ros::TransformListener(tf_buffer)
		);
		return true;
	}

	void reconstructStartingPlanPoints(const autoware_msgs::Lane& msg,
			geometry_msgs::PoseStamped& pose, int offset, int skip = 0)
	{
		original_velocity_profile.clear();
		if (msg.waypoints.size() >= 2)
		{
			starting_plan_points.clear();
			number_of_trajectory_points = 1;
			geometry_msgs::PoseStamped _pose;
			tf2::doTransform(pose, _pose, transform_current_pose);
			starting_plan_points.push_back(_pose);
			for (int i = offset + skip; i < trajectory_slicer.getLookaheadIndex(); i++)
			{
				geometry_msgs::PoseStamped _wp;
				tf2::doTransform(msg.waypoints[i].pose, _wp, transform_current_pose);
				starting_plan_points.push_back(std::move(_wp));
				original_velocity_profile.push_back(msg.waypoints[i].twist);

				number_of_trajectory_points++;
			}

			//starting_plan_points.push_backfront() = _pose;
			number_of_trajectory_points++;
		}
	}
public:
	Abstract_RosLocalPlanner(): number_of_trajectory_points(0){}

	virtual void executePlannerMethods() = 0;
	virtual void relayCycle() = 0;
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
