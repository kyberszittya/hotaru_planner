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
		std::pair<std::string, std::unique_ptr<PortMonitorState>>(
				name,std::unique_ptr<PortMonitorState>(
						new PortMonitorState(freq))
		)
	);
}

void PortStateMonitor::updateTimestamp(std::string name, unsigned long long timestamp)
{
	if (timestamp - signal_timestamps[name]->timestamp > signal_timestamps[name]->freq)
	{
		signal_timestamps[name]->valid = false;
	}
	else
	{
		signal_timestamps[name]->valid = true;
	}
	signal_timestamps[name]->timestamp = timestamp;
}

bool PortStateMonitor::isReady()
{
	for (const auto& c: signal_timestamps)
	{
		if (!c.second->valid)
		{
			return false;
		}
	}
	return true;
}

void PortStateMonitor::updateTimeout(unsigned long long current_time)
{
	for (const auto& c: signal_timestamps)
	{
		if (current_time - c.second->timestamp > c.second->freq)
		{
			c.second->valid = false;
		}
	}
}

void PortStateMonitor::timeoutAsync()
{
	/*
	while(!f_isStopped())
	{
		//waitClock();
	}
	*/
}


}
