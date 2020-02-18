/*
 * port_monitor.hpp
 *
 *  Created on: Feb 10, 2020
 *      Author: kyberszittya
 */

#ifndef INCLUDE_REI_COMMON_STATE_MACHINE_PORT_MONITOR_HPP_
#define INCLUDE_REI_COMMON_STATE_MACHINE_PORT_MONITOR_HPP_

#include <map>
#include <memory>
#include <functional>

namespace rei
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
	virtual ~PortStateMonitor() = 0;

	void addPort(std::string name, const unsigned long freq);

	void updateTimestamp(std::string name, unsigned long long timestamp);

	bool isReady();

	virtual void waitClock(double duration_sec) = 0;

	void updateTimeout(unsigned long long current_time);

	void timeoutAsync();

};

}



#endif /* INCLUDE_HOTARU_COMMON_STATE_MACHINE_PORT_MONITOR_HPP_ */
