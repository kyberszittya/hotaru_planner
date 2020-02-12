/*
 * port_monitor.hpp
 *
 *  Created on: Feb 10, 2020
 *      Author: kyberszittya
 */

#ifndef INCLUDE_HOTARU_COMMON_STATE_MACHINE_PORT_MONITOR_HPP_
#define INCLUDE_HOTARU_COMMON_STATE_MACHINE_PORT_MONITOR_HPP_

#include <map>
#include <memory>
#include <functional>

namespace hotaru
{

struct PortMonitorState
{
	const unsigned long freq;
	unsigned long long timestamp;
	bool valid;

	PortMonitorState(const unsigned long freq):
		freq(freq),
		timestamp(0), valid(false){}
};

class PortStateMonitor
{
private:
	// A map of port identifier and the latest timestamp
	std::map<std::string, std::unique_ptr<PortMonitorState>> signal_timestamps;
	// Assign a function to check stop status
	std::function<bool(void)> f_isStopped();
public:



	void addPort(std::string name, const unsigned long freq)
	{
		signal_timestamps.insert(
			std::pair<std::string, std::unique_ptr<PortMonitorState>>(
					name,std::unique_ptr<PortMonitorState>(
							new PortMonitorState(freq))
			)
		);
	}

	void updateTimestamp(std::string name, unsigned long long timestamp)
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

	bool isReady()
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

	virtual void waitClock(double duration_sec) = 0;

	void updateTimeout(unsigned long long current_time)
	{
		for (const auto& c: signal_timestamps)
		{
			if (current_time - c.second->timestamp > c.second->freq)
			{
				c.second->valid = false;
			}
		}
	}

	void timeoutAsync()
	{
		while(!f_isStopped())
		{
			//waitClock();
		}
	}
};

}



#endif /* INCLUDE_HOTARU_COMMON_STATE_MACHINE_PORT_MONITOR_HPP_ */
