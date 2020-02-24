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

#include <rei_statemachine_library/testing/test_common.hpp>


namespace rei
{

constexpr unsigned long nsecs_l = 1000000000;

class RosSyncStateMachine
{
private:
	const std::string name;
	std::shared_ptr<ros::NodeHandle> nh;
protected:
	std::shared_ptr<PortStateMonitorRos> port_state_monitor;
	std::shared_ptr<RosCommunicationGraphNotifier> notifier;
	std::shared_ptr<SyncStateMachine> sync_state_machine;

public:
	RosSyncStateMachine(std::shared_ptr<ros::NodeHandle> nh,
			const std::string name): nh(nh), name(name){

	}

	// Let us allow to initialize everything externally
	RosSyncStateMachine(std::shared_ptr<ros::NodeHandle> nh,
			std::shared_ptr<SyncStateMachine> sync_state_machine,
			std::shared_ptr<PortStateMonitorRos> port_state_monitor,
			std::shared_ptr<RosCommunicationGraphNotifier> notifier,
			const std::string name): nh(nh),
					port_state_monitor(port_state_monitor),
					notifier(notifier),
					sync_state_machine(sync_state_machine),
					name(name){
	}

	bool initialize()
	{

		// If the state machine hasn't been initialized externally, let's initialize it
		if (sync_state_machine==nullptr)
		{
			port_state_monitor= std::shared_ptr<PortStateMonitorRos>(new PortStateMonitorRos());
			if (port_state_monitor == nullptr)
			{
				return false;
			}
			std::unique_ptr<RosSyncStateGuard> guard(new RosSyncStateGuard());
			guard->setMonitor(port_state_monitor);
			notifier = std::shared_ptr<RosCommunicationGraphNotifier>(
					new RosCommunicationGraphNotifier(name, nh));
			if (notifier == nullptr)
			{
				return false;
			}
			notifier->initialize();
			sync_state_machine = std::shared_ptr<SyncStateMachine>(
					new SyncStateMachine(notifier,
							std::move(guard)));
		}
		if (sync_state_machine == nullptr)
		{
			ROS_ERROR("SYNC STATE MACHINE uninitialized");
			return false;
		}
		port_state_monitor->setSyncStateMachine(sync_state_machine);
		sync_state_machine->setStartFunction(std::bind(&RosSyncStateMachine::startRos, this));
		sync_state_machine->start();
		return true;
	}

	void startRos()
	{
		ROS_INFO_STREAM("Initializing sync_state_machine: " << name);
	}

	void addTopicGuard(std::string name, double estimated_frequency)
	{
		port_state_monitor->addPort(name,
				static_cast<unsigned long>(estimated_frequency*nsecs_l));
	}

	void stepMessageTopic(std::string name, const std_msgs::Header& header)
	{
		port_state_monitor->updateTimestamp(name, header.stamp.toNSec());
	}

	void step()
	{
		sync_state_machine->stepstatemachine();
	}

	bool isReady()
	{
		return sync_state_machine->isStarted();
	}

};

}


#endif /* INCLUDE_REI_STATEMACHINE_LIBRARY_ROS_ROS_SYNC_STATE_MACHINE_HPP_ */
