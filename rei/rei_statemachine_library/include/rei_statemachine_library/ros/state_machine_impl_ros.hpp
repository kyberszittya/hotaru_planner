/*
 * state_machine_impl_ros.hpp
 *
 *  Created on: Feb 10, 2020
 *      Author: kyberszittya
 */

#ifndef INCLUDE_REI_COMMON_STATE_MACHINE_STATE_MACHINE_IMPL_ROS_HPP_
#define INCLUDE_REI_COMMON_STATE_MACHINE_STATE_MACHINE_IMPL_ROS_HPP_



#include <ros/ros.h>
#include <rei_statemachine_library/portsync_state_machine/sync_state_machine.hpp>
#include <rei_statemachine_library/port_monitor/port_monitor.hpp>

namespace rei
{

class PortStateMonitorRos: rei::PortStateMonitor
{
public:
	virtual void waitClock(double duration_sec)
	{
		ros::Duration(duration_sec).sleep();
	}
};

class RosSyncStateGuard: public rei::Interface_GuardSyncStates
{
protected:
	std::shared_ptr<PortStateMonitorRos> monitor_state;
public:
	void setMonitor(std::shared_ptr<PortStateMonitorRos> monitor_state)
	{
		this->monitor_state = monitor_state;
	}

	virtual bool guard_StartedState()
	{
		return monitor_state->isReady();
	}

	virtual bool guard_WaitingState()
	{
		return !monitor_state->isReady();
	}

	virtual bool guard_Stopped()
	{
		return true;
	}
};

}

#endif /* INCLUDE_HOTARU_COMMON_STATE_MACHINE_STATE_MACHINE_IMPL_ROS_HPP_ */
