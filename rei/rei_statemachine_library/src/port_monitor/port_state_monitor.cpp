/*
 * port_state_monitor.cpp
 *
 *  Created on: Feb 18, 2020
 *      Author: kyberszittya
 */

#include <rei_statemachine_library/port_monitor/port_monitor.hpp>


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

void PortStateMonitor::updateTimestamp(std::string portmonitorstate, unsigned long long timestamp)
{
	updateTimestamp(signal_timestamps[portmonitorstate], timestamp);
}



void PortStateMonitor::checkStateTimestamp(std::shared_ptr<PortMonitorState> portmonitorstate, unsigned long long timestamp)
{
	switch (portmonitorstate->state)
	{
		case PortMonitorState_State::FRESH:
		{
			if (timestamp - portmonitorstate->timestamp >= portmonitorstate->freq)
			{
				portmonitorstate->state = PortMonitorState_State::TIMEOUT;
			}
			break;
		}
		case PortMonitorState_State::TIMEOUT:
		{
			// TODO: I have a strange feeling that it will not work as expected, revise it!
			if (timestamp - portmonitorstate->timestamp < portmonitorstate->freq)
			{
				portmonitorstate->state = PortMonitorState_State::FRESH;
			}
			break;
		}
	}

}

void PortStateMonitor::checkAllStatesTimestamp(unsigned long long timestamp)
{
	for (const auto& v: signal_timestamps)
	{
		checkStateTimestamp(v.second, timestamp);
	}
}

void PortStateMonitor::updateTimestamp(std::shared_ptr<PortMonitorState> portmonitorstate, unsigned long long timestamp)
{
	switch (portmonitorstate->state)
	{
		case PortMonitorState_State::UNINITIALIZED:
		{
			portmonitorstate->state = PortMonitorState_State::FRESH;
			break;
		}
	}
	checkAllStatesTimestamp(timestamp);
	portmonitorstate->timestamp = timestamp;
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
