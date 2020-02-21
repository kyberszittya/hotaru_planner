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

#include <rei_monitoring_msgs/ReiMonitorSignal.h>

namespace rei
{

class PortStateMonitorRos: public rei::PortStateMonitor
{
private:
	std::shared_ptr<SyncStateMachine> sm;
public:
	virtual void react_TimeOut(unsigned long long timestamp)
	{
		using namespace rei::sync_signals;
		std::shared_ptr<SignalMessageTimeOut> sig_(new SignalMessageTimeOut(timestamp));
		sm->propagateSignal(sig_);
	}

	virtual void react_Fresh(unsigned long long timestamp)
	{
		using namespace rei::sync_signals;
		std::shared_ptr<SignalMessageTimeOut> sig_(new SignalMessageTimeOut(timestamp));
		if (isReady())
		{
			sm->propagateSignal(sig_);
		}
	}

	void setSyncStateMachine(std::shared_ptr<SyncStateMachine> sm)
	{
		this->sm = sm;

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
