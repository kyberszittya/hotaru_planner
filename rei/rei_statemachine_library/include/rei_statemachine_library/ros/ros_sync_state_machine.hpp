/*
 * ros_sync_state_machine.hpp
 *
 *  Created on: Feb 21, 2020
 *      Author: kyberszittya
 */

#ifndef INCLUDE_REI_STATEMACHINE_LIBRARY_ROS_ROS_SYNC_STATE_MACHINE_HPP_
#define INCLUDE_REI_STATEMACHINE_LIBRARY_ROS_ROS_SYNC_STATE_MACHINE_HPP_

#include <rei_statemachine_library/portsync_state_machine/sync_state_machine.hpp>
#include <rei_statemachine_library/port_monitor/port_monitor.hpp>
#include <rei_statemachine_library/ros/state_machine_impl_ros.hpp>
#include <rei_statemachine_library/ros/ros_sync_state_machine.hpp>
#include <rei_statemachine_library/ros/ros_notifier.hpp>

#include <std_msgs/Header.h>
#include <ros/ros.h>


namespace rei
{

constexpr unsigned long nsecs_l = 1000000000;

class RosSyncStateMachine
{
private:
	std::string name;
	std::shared_ptr<ros::NodeHandle> nh;
protected:
	std::shared_ptr<PortStateMonitorRos> port_state_monitor;
	std::shared_ptr<SyncStateMachine> sync_state_machine;
	std::shared_ptr<RosCommunicationGraphNotifier> notifier;
public:
	RosSyncStateMachine(std::shared_ptr<ros::NodeHandle> nh,
			const std::string& name): name(name), nh(nh){

	}

	void initialize()
	{
		port_state_monitor= std::shared_ptr<PortStateMonitorRos>(new PortStateMonitorRos());
		std::unique_ptr<RosSyncStateGuard> guard(new RosSyncStateGuard());
		guard->setMonitor(port_state_monitor);
		notifier = std::shared_ptr<RosCommunicationGraphNotifier>(
				new RosCommunicationGraphNotifier(name, nh));
		sync_state_machine.reset(new SyncStateMachine(notifier,
				std::move(guard)));
		sync_state_machine->start();
	}

	void addTopicGuard(std::string name, double estimated_frequency)
	{
		port_state_monitor->addPort(name,
				static_cast<unsigned long>(estimated_frequency*nsecs_l));
	}

	void stepMessageTopic(std::string name, std_msgs::Header& header)
	{
		port_state_monitor->updateTimestamp(name, header.stamp.toNSec());

	}

};

}


#endif /* INCLUDE_REI_STATEMACHINE_LIBRARY_ROS_ROS_SYNC_STATE_MACHINE_HPP_ */
