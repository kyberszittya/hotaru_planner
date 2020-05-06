/*
 * hotaru_planner_node_ros1.hpp
 *
 *  Created on: Apr 2, 2020
 *      Author: kyberszittya
 */

#ifndef INCLUDE_HOTARU_NODE_ELEMENTS_HOTARU_PLANNER_NODE_ROS1_HPP_
#define INCLUDE_HOTARU_NODE_ELEMENTS_HOTARU_PLANNER_NODE_ROS1_HPP_

#include "gen_interfaceros_abstracthotaruplannernode.h"
#include "abstract_planner_component.hpp"

#include "state_machine/trajectory_statemachine.hpp"


#include <memory>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace hotaru
{

class HotaruPlannerNodeRos;

class PlannerTransformStateTf2: public PlannerTransformState
{
	tf2_ros::Buffer buffer;
	tf2_ros::TransformListener listener;
	// TODO: TF seems to be a little time costly function let's use it in another thread
public:
	PlannerTransformStateTf2(const std::string base_frame, const std::string global_frame):
		PlannerTransformState(base_frame, global_frame),
		listener(buffer){}

	bool getStampedTransform(geometry_msgs::TransformStamped& msg, geometry_msgs::TransformStamped& inverse_msg)
	{
		try
		{
			// Check for the global to robot transformation
			msg = buffer.lookupTransform(base_frame, global_frame, ros::Time::now(), ros::Duration(1.0));
			// Check for the inverse as well
			inverse_msg = buffer.lookupTransform(global_frame, base_frame, ros::Time::now(), ros::Duration(1.0));
			return true;
		}
		catch(tf2::LookupException& le)
		{
			ROS_WARN_STREAM("LOOKUP EXCEPTION: " << le.what());
		}
		catch(tf2::ConnectivityException& ce)
		{
			ROS_WARN_STREAM("CONNECTIVITY EXCPETION: " << ce.what());
		}
		catch(tf2::ExtrapolationException& ce)
		{
			ROS_WARN_STREAM("EXTRAPOLATION EXCEPTION: " << ce.what());
		}
		return false;
	}

	/**
	 * @brief: Get pose in defined frame
	 * @precondition: header is defined
	 */
	static bool getPoseInFrame(const geometry_msgs::Pose& pose_in,
			geometry_msgs::TransformStamped& transform_stamped,
			geometry_msgs::Pose& pose_out)
	{
		tf2::doTransform(pose_in, pose_out, transform_stamped);
		return true;
	}



};

class HotaruPlannerNodeRos: public HotaruPlannerNode<hotaru_msgs::Waypoint>,
	public InterfaceRos_AbstractHotaruPlannerNode
{
private:
protected:
	// Timers
	ros::Timer timer_planner;  										///< Execute planner periodically
	std::unique_ptr<LocalPlannerStateMachine> sm_behav_planner;
	geometry_msgs::TransformStamped transform_base;
	geometry_msgs::TransformStamped inv_transform_base;
	// TF
	PlannerTransformStateTf2 tf_planner_state;
	// Mutex
	std::mutex mtx_planner;
	std::mutex mtx_trajectory_update;
	// Thread
	virtual void startAfterSynchronization() = 0;
	virtual void timeoutSynchronization() = 0;
public:

	HotaruPlannerNodeRos(
			std::shared_ptr<ros::NodeHandle> private_nh,
			std::shared_ptr<ros::NodeHandle> nh,
			std::string base_frame, std::string target_frame
			):
		InterfaceRos_AbstractHotaruPlannerNode(private_nh, nh),
		tf_planner_state(base_frame, target_frame){}
};

}

#endif /* INCLUDE_HOTARU_NODE_ELEMENTS_HOTARU_PLANNER_NODE_ROS1_HPP_ */
