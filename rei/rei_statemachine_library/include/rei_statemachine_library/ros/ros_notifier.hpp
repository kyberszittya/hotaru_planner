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
#include <rei_monitoring_msgs/ReiMonitorSignal.h>

namespace rei
{

class RosCommunicationGraphNotifier: public Interface_CommunicationGraphNotifier
{
private:
	std::string node_name;
	std::shared_ptr<ros::NodeHandle> nh;
	rei_monitoring_msgs::ReiMonitorSignal msg_sig;
	ros::Publisher pub_sig_id;
public:
	RosCommunicationGraphNotifier(std::string& node_name,
			std::shared_ptr<ros::NodeHandle> nh):
		nh(nh) {}

	void initialize()
	{
		pub_sig_id = nh->advertise<rei_monitoring_msgs::ReiMonitorSignal>(
				node_name+"/sync_state_machine/current_state", 10);
	}


	virtual void notifyCommunicationGraph(std::shared_ptr<AbstractSignalInterface> sig)
	{
		msg_sig.sig_name = "sync_signal";
		msg_sig.sig_id = sig->getId();
		pub_sig_id.publish(msg_sig);
	}
};

}

#endif
