/*
 * ros_notifier.cpp
 *
 *  Created on: Feb 21, 2020
 *      Author: kyberszittya
 */

#ifndef INCLUDE_REI_COMMON_STATE_MACHINE_ROS_NOTIFIER_HPP_
#define INCLUDE_REI_COMMON_STATE_MACHINE_ROS_NOTIFIER_HPP_

#include <rei_statemachine_library/abstract_statemachine_definitions.hpp>
#include <ros/ros.h>
#include <rei_monitoring_msgs/ReiStateMachineTransitionSignal.h>

namespace rei
{

std::string translateSignalToName(std::shared_ptr<AbstractSignalInterface> _sig);


class RosCommunicationGraphNotifier: public Interface_CommunicationGraphNotifier
{
private:
	std::string node_name;
	std::shared_ptr<ros::NodeHandle> nh;
	rei_monitoring_msgs::ReiStateMachineTransitionSignal msg_sig;
	ros::Publisher pub_sig_id;
public:
	RosCommunicationGraphNotifier(std::string node_name,
			std::shared_ptr<ros::NodeHandle> nh):
				node_name(node_name), nh(nh) {}

	void initialize()
	{
		ROS_INFO_STREAM("Initializing ROS communication notifier: "
				<< node_name+"/sync_state_machine/current_state");
		pub_sig_id = nh->advertise<rei_monitoring_msgs::ReiStateMachineTransitionSignal>(
				node_name+"/sync_state_machine/current_state", 10);
	}


	virtual void notifyCommunicationGraph(std::shared_ptr<AbstractSignalInterface> sig)
	{
		msg_sig.header.stamp = ros::Time::now();
		msg_sig.signal_name = translateSignalToName(sig);
		msg_sig.sig_id = sig->getId();
		pub_sig_id.publish(msg_sig);
	}
};

}

#endif
