/*
 * ros_sync_utils.cpp
 *
 *  Created on: Feb 23, 2020
 *      Author: kyberszittya
 */

#include <rei_statemachine_library/abstract_statemachine_definitions.hpp>
#include <ros/ros.h>
#include <rei_monitoring_msgs/ReiStateMachineTransitionSignal.h>

namespace rei
{


std::string translateSignalToName(std::shared_ptr<AbstractSignalInterface> _sig)
{
	switch(_sig->getId())
	{
		case 0x01: {
			return "SignalAllStateMessagesReceived";
		}
		case 0x02: {
			return "SignalMessageTimeOut";
		}
		case 0x03: {
			return "SignalTerminationRequest";
		}
		default: {
			return "";
		}
	}
}

}
