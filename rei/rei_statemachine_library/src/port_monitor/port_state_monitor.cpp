/*
 * port_state_monitor.cpp
 *
 *  Created on: Feb 18, 2020
 *      Author: kyberszittya
 */

#include <rei_statemachine_library/port_monitor/port_monitor.hpp>

#include <iostream>

#include <ros/ros.h>

namespace rei
{

PortStateMonitor::~PortStateMonitor(){}

void PortStateMonitor::addPort(std::string name, const unsigned long freq)
{
	signal_timestamps.insert(
		std::pair<std::string, std::shared_ptr<PortMonitorState>>(
				name,std::shared_ptr<PortMonitorState>(
						new PortMonitorState(name, freq))
		)
	);
}

bool PortMonitorState::isFresh() const
{
	return state == PortMonitorState_State::FRESH;
}

void PortStateMonitor::updateTimestamp(std::string portmonitorstate, const unsigned long long timestamp)
{
	updateTimestamp(signal_timestamps[portmonitorstate], timestamp);
}



void PortStateMonitor::checkStateTimestamp(std::shared_ptr<PortMonitorState> portmonitorstate, const unsigned long long timestamp)
{
	unsigned long long dt;
	if (portmonitorstate->timestamp > timestamp)
	{
		dt = 0;
	}
	else
	{
		dt = timestamp - portmonitorstate->timestamp;
	}
	switch (portmonitorstate->state)
	{
		case PortMonitorState_State::FRESH:
		{
			if (dt >= portmonitorstate->freq)
			{
				ROS_WARN_STREAM("Timeout on port: " << portmonitorstate->name << ", expected sample time [ns]: " << portmonitorstate->freq);
				portmonitorstate->state = PortMonitorState_State::TIMEOUT;
				react_TimeOut(timestamp);
			}
			break;
		}
		case PortMonitorState_State::TIMEOUT:
		{
			// TODO: I have a strange feeling that it will not work as expected, revise it!

			if (dt < portmonitorstate->freq)
			{
				portmonitorstate->state = PortMonitorState_State::FRESH;
				react_Fresh(timestamp);
			}
			break;
		}
	}

}

void PortStateMonitor::checkAllStatesTimestamp(const unsigned long long timestamp)
{
	for (const auto& v: signal_timestamps)
	{
		checkStateTimestamp(v.second, timestamp);
	}
}

void PortStateMonitor::updateTimestamp(std::shared_ptr<PortMonitorState> portmonitorstate, const unsigned long long timestamp)
{
	checkStateTimestamp(portmonitorstate, timestamp);
	portmonitorstate->timestamp = timestamp;
	switch (portmonitorstate->state)
	{
		case PortMonitorState_State::UNINITIALIZED:
		{
			portmonitorstate->state = PortMonitorState_State::FRESH;
			react_Fresh(timestamp);
			break;
		}
	}
}

bool PortStateMonitor::isReady()
{
	for (const auto& c: signal_timestamps)
	{
		if (!c.second->isFresh())
		{
			return false;
		}
	}
	return true;
}



}
