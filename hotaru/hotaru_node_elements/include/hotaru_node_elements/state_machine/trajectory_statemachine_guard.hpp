/*
 * trajectory_statemachine_guard.hpp
 *
 *  Created on: Apr 3, 2020
 *      Author: kyberszittya
 */

#ifndef INCLUDE_HOTARU_NODE_ELEMENTS_STATE_MACHINE_TRAJECTORY_STATEMACHINE_GUARD_HPP_
#define INCLUDE_HOTARU_NODE_ELEMENTS_STATE_MACHINE_TRAJECTORY_STATEMACHINE_GUARD_HPP_

#include <geometry_msgs/Point.h>

#include <hotaru_node_elements/state_machine/trajectory_statemachine.hpp>
#include <rei_statemachine_library/portsync_state_machine/sync_state_machine.hpp>

class SyncGuardLocalPlanner: public hotaru::Interface_GuardLocalPlanner
{
protected:
	std::shared_ptr<rei::SyncStateMachine> sync_sm;

	geometry_msgs::Point lookahead_point;
	geometry_msgs::Point current_point;
	int lookahead_index;
	int current_lookahead_index;
	int minimal_velocity_distance_index;
public:
	SyncGuardLocalPlanner(std::shared_ptr<rei::SyncStateMachine> sync_sm):
		sync_sm(sync_sm),
		lookahead_index(0),
		current_lookahead_index(0),
		minimal_velocity_distance_index(0)
	{
	}

	virtual bool guard_Relay2ReplanningState() override
	{
		return sync_sm->isStarted();
	}

	virtual bool guard_Replanning2RelayState() override
	{
		return true;

	}
	virtual bool guard_Relay2Waiting() override
	{
		return sync_sm->isStarted();
	}
	virtual bool guard_Waiting2Relay() override
	{
		return sync_sm->isStarted();
	}
	void setLookaheadPoint(geometry_msgs::Point msg,
			int speed_minimal_index,
			int lookahead_index,
			int trajectory_length)
	{
		minimal_velocity_distance_index = speed_minimal_index;
		lookahead_point = msg;
		this->lookahead_index = std::min(lookahead_index, trajectory_length);
	}

	void setCurrentLookaheadIndex(int index)
	{
		current_lookahead_index = index;
	}

	void setCurrentPoint(geometry_msgs::Point msg)
	{
		current_point = msg;
	}
};


#endif /* INCLUDE_HOTARU_NODE_ELEMENTS_STATE_MACHINE_TRAJECTORY_STATEMACHINE_GUARD_HPP_ */
