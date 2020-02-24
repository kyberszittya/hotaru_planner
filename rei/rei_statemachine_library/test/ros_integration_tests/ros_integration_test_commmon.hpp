/*
 * ros_integration_test_commmon.hpp
 *
 *  Created on: Feb 22, 2020
 *      Author: kyberszittya
 */

#ifndef TEST_ROS_INTEGRATION_TESTS_ROS_INTEGRATION_TEST_COMMMON_HPP_
#define TEST_ROS_INTEGRATION_TESTS_ROS_INTEGRATION_TEST_COMMMON_HPP_
#include <ros/ros.h>
#include <gtest/gtest.h>

#include <geometry_msgs/PoseStamped.h>
#include <rei_monitoring_msgs/ReiStateMachineTransitionSignal.h>

class StateMachineStateReader
{
private:
	std::shared_ptr<ros::NodeHandle> nh;
	ros::Subscriber sub_state;
	rei_monitoring_msgs::ReiStateMachineTransitionSignal msg_signal;
public:
	StateMachineStateReader(std::shared_ptr<ros::NodeHandle> nh): nh(nh)
	{

	}

	void init()
	{
		sub_state = nh->subscribe("/simple_sync_state/sync_state_machine/current_state", 10,
				&StateMachineStateReader::cbStateTransitionSignal, this);
	}

	void cbStateTransitionSignal(const rei_monitoring_msgs::ReiStateMachineTransitionSignal::ConstPtr& msg)
	{
		msg_signal = *msg;
	}

	rei_monitoring_msgs::ReiStateMachineTransitionSignal getSignal() const
	{
		return msg_signal;
	}
};


#endif /* TEST_ROS_INTEGRATION_TESTS_ROS_INTEGRATION_TEST_COMMMON_HPP_ */
