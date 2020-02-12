/*
 * signal.hpp
 *
 *  Created on: Jan 30, 2020
 *      Author: kyberszittya
 */

#ifndef INCLUDE_HOTARU_COMMON_STATE_MACHINE_SYNC_SIGNAL_HPP_
#define INCLUDE_HOTARU_COMMON_STATE_MACHINE_SYNC_SIGNAL_HPP_

#include "abstract_signal_definitions.hpp"

namespace hotaru
{
namespace sync_signals
{
class SignalAllStateMessagesReceived: public AbstractSignal<0x01>
{
private:
public:
	SignalAllStateMessagesReceived(const unsigned long long timestamp):
		AbstractSignal(timestamp){}
};

class SignalMessageTimeOut: public AbstractSignal<0x02>
{
private:
public:
	SignalMessageTimeOut(const unsigned long long timestamp):
		AbstractSignal(timestamp){}
};

class SignalTerminationRequest: public AbstractSignal<0xFFFF>
{
private:
public:
	SignalTerminationRequest(const unsigned long long timestamp):
		AbstractSignal(timestamp){}

};

}

namespace port_signals
{

class SignalPortReceived: public AbstractSignal<0x10>
{
private:
public:
	SignalPortReceived(const unsigned long long timestamp):
			AbstractSignal(timestamp){}
};

class SignalPortTimeout: public AbstractSignal<0x11>
{
private:
public:
	SignalPortTimeout(const unsigned long long timestamp):
			AbstractSignal(timestamp){}
};

}

}


#endif /* INCLUDE_HOTARU_COMMON_STATE_MACHINE_SYNC_SIGNAL_HPP_ */
