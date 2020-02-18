/*
 * trajectory_signal.hpp
 *
 *  Created on: Feb 17, 2020
 *      Author: kyberszittya
 */

#ifndef INCLUDE_HOTARU_COMMON_STATE_MACHINE_TRAJECTORY_SIGNAL_HPP_
#define INCLUDE_HOTARU_COMMON_STATE_MACHINE_TRAJECTORY_SIGNAL_HPP_

#include <rei_statemachine_library/abstract_signal_definitions.hpp>

namespace hotaru
{

namespace trajectory_signals
{

class SignalReplanningTrajectory: public rei::AbstractSignal<0x20>
{
private:
public:
	SignalReplanningTrajectory(const unsigned long long timestamp):
		AbstractSignal(timestamp){}

};

class SignalNoObstacleDetected: public rei::AbstractSignal<0x21>
{
private:
public:
	SignalNoObstacleDetected(const unsigned long long timestamp):
		AbstractSignal(timestamp){}
};

class SignalLastWaypointReached: public rei::AbstractSignal<0x22>
{
private:
public:
	SignalLastWaypointReached(const unsigned long long timestamp):
			AbstractSignal(timestamp){}
};

class SignalNewGlobalPlan: public rei::AbstractSignal<0x23>
{
private:
public:
	SignalNewGlobalPlan(const unsigned long long timestamp):
		AbstractSignal(timestamp){}
};

}

}
#endif /* INCLUDE_HOTARU_COMMON_STATE_MACHINE_TRAJECTORY_SIGNAL_HPP_ */
