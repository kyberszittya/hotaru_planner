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
class PortStateMonitor;

enum class PortMonitorState_State { UNINITIALIZED, FRESH, TIMEOUT };

struct PortMonitorState
{
	const unsigned long freq;
	unsigned long long timestamp;
	PortMonitorState_State state;
	std::string name;

	PortMonitorState(std::string& name, const unsigned long freq):
		name(name),
		freq(freq),
		timestamp(0), state(PortMonitorState_State::UNINITIALIZED){}

	bool isFresh() const;

	friend PortStateMonitor;
};

class PortStateMonitor
{
private:
	// A map of port identifier and the latest timestamp
	std::map<std::string, std::shared_ptr<PortMonitorState>> signal_timestamps;

public:
	PortStateMonitor()
	{}



	virtual ~PortStateMonitor() = 0;

	void addPort(std::string name, const unsigned long freq);

	void updateTimestamp(std::string name, unsigned long long timestamp);

	void updateTimestamp(std::shared_ptr<PortMonitorState> portmonitorstate, unsigned long long timestamp);

	void checkAllStatesTimestamp(unsigned long long timestamp);

	void checkStateTimestamp(std::shared_ptr<PortMonitorState> portmonitorstate, unsigned long long timestamp);

	virtual void react_TimeOut(unsigned long long timestamp) = 0;
	virtual void react_Fresh(unsigned long long timestamp) = 0;

	bool isReady();


};

}



#endif /* INCLUDE_HOTARU_COMMON_STATE_MACHINE_PORT_MONITOR_HPP_ */
