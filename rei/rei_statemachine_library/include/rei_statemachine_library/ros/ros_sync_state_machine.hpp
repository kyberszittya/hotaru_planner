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
	const double step_hz_rate;
protected:
	std::shared_ptr<PortStateMonitorRos> port_state_monitor;
	std::shared_ptr<RosCommunicationGraphNotifier> notifier;
	std::shared_ptr<SyncStateMachine> sync_state_machine;
	// Timer to step and update messages
	ros::Timer timer_syn_sm;
	// Exclusive mutex to step
	std::shared_ptr<std::mutex> mtx_sm;
public:
	RosSyncStateMachine(std::shared_ptr<ros::NodeHandle> nh,
			const std::string name, const double step_hz_rate = 60.0):
				nh(nh), name(name), step_hz_rate(step_hz_rate){

	}

	// Let us allow to initialize everything externally
	RosSyncStateMachine(std::shared_ptr<ros::NodeHandle> nh,
			std::shared_ptr<SyncStateMachine> sync_state_machine,
			std::shared_ptr<PortStateMonitorRos> port_state_monitor,
			std::shared_ptr<RosCommunicationGraphNotifier> notifier,
			const std::string name, const double step_hz_rate = 60.0): nh(nh),
					port_state_monitor(port_state_monitor),
					notifier(notifier),
					sync_state_machine(sync_state_machine),
					name(name), step_hz_rate(step_hz_rate){
	}

	void cbTimerSynSm(const ros::TimerEvent& e)
	{
		uint64_t time = e.current_real.toNSec();
		port_state_monitor->checkAllStatesTimestamp(time);
		mtx_sm->lock();
		sync_state_machine->stepstatemachine();
		mtx_sm->unlock();
		/// If waiting, inform whether there are topics to be waited of
		if (sync_state_machine->isWaiting())
		{
			if (port_state_monitor->isReady())
			{
				ROS_ERROR("Unexpected error attempting to react with sync sm");
				port_state_monitor->react_Fresh(time);
			}
			else
			{
				for (const auto& v: port_state_monitor->getTimeoutPorts())
				{
					ROS_WARN_STREAM("Waiting for input on: " << v);
				}
			}
		}
	}

	bool initialize()
	{
		// Initialize mutex
		mtx_sm = std::make_shared<std::mutex>();
		if (mtx_sm==nullptr)
		{
			return false;
		}
		// If the state machine hasn't been initialized externally, let's initialize it
		if (sync_state_machine==nullptr)
		{
			port_state_monitor= std::shared_ptr<PortStateMonitorRos>(new PortStateMonitorRos(mtx_sm));
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
		timer_syn_sm = nh->createTimer(
				ros::Duration(1.0/step_hz_rate),
				&RosSyncStateMachine::cbTimerSynSm, this);
		ROS_INFO_STREAM("Started ROS timer for Sync state machine");
	}

	void addTopicGuard(std::string name, double estimated_frequency)
	{
		port_state_monitor->addPort(name,
				static_cast<unsigned long>(estimated_frequency*nsecs_l));
	}

	void stepMessageTopic(std::string name, const ros::Time& header)
	{
		port_state_monitor->updateTimestamp(name, header.toNSec());
	}

	void stepMessageTopic(std::string name, const std_msgs::Header& header)
	{
		port_state_monitor->updateTimestamp(name, header.stamp.toNSec());
	}

	void step()
	{
		mtx_sm->lock();
		sync_state_machine->stepstatemachine();
		mtx_sm->unlock();
	}

	bool isReady()
	{
		return sync_state_machine->isStarted();
	}




};

}


#endif /* INCLUDE_REI_STATEMACHINE_LIBRARY_ROS_ROS_SYNC_STATE_MACHINE_HPP_ */
