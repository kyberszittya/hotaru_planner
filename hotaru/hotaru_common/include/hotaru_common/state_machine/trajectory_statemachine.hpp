/*
 * trajectory_statemachine.hpp
 *
 *  Created on: Feb 16, 2020
 *      Author: kyberszittya
 */

#ifndef INCLUDE_HOTARU_COMMON_STATE_MACHINE_TRAJECTORY_STATEMACHINE_HPP_
#define INCLUDE_HOTARU_COMMON_STATE_MACHINE_TRAJECTORY_STATEMACHINE_HPP_


#include <rei_statemachine_library/abstract_statemachine_definitions.hpp>
#include "trajectory_signal.hpp"


namespace hotaru
{

enum class LocalPlannerStateMachine_States {
	PSEUDO_START,
	RELAY,
	REPLANNING,
	WAITING,
	PSEUDO_EXIT
};

class Interface_GuardLocalPlanner: public rei::Abstract_StateMachineGuard
{
public:
	virtual bool guard_Relay2ReplanningState() = 0;
	virtual bool guard_Replanning2RelayState() = 0;
	virtual bool guard_Relay2Waiting() = 0;
	virtual bool guard_Waiting2Relay() = 0;
};

class Interface_ReactTrajectorySignals
{
public:
	//virtual ~Interface_ReactTrajectorySignals() = 0;
	// React to all state messages received signal
	virtual void reactSig_ReplanningTrajectory(std::shared_ptr<rei::AbstractSignalInterface> sig) = 0;
	// React to timeout message
	virtual void reactSig_SignalNoObstacleDetected(std::shared_ptr<rei::AbstractSignalInterface> sig) = 0;
	// React to last waypoint reached
	virtual void reactSig_LastWaypointReached(std::shared_ptr<rei::AbstractSignalInterface> sig) = 0;
	// React to new global waypoint
	virtual void reactSig_NewGlobalPlan(std::shared_ptr<rei::AbstractSignalInterface> sig) = 0;
};

class LocalPlannerStateMachine: public rei::AbstractStateMachine<
	LocalPlannerStateMachine_States, Interface_GuardLocalPlanner>,
	public Interface_ReactTrajectorySignals
{
private:
protected:
	virtual void handle_start() override
	{
		switch(state)
		{
			case LocalPlannerStateMachine_States::PSEUDO_START:
			{
				state = LocalPlannerStateMachine_States::RELAY;
				break;
			}
		}
	}

	virtual void handle_startError() override
	{
		throw  rei::StateMachineStartFailure("local_planner_state_machine", "start");
	}

public:
	LocalPlannerStateMachine(
		std::shared_ptr<rei::Interface_CommunicationGraphNotifier> graph_notifier,
		std::unique_ptr<Interface_GuardLocalPlanner> guard_syncstate):
			AbstractStateMachine(LocalPlannerStateMachine_States::PSEUDO_START,
					graph_notifier,
					std::move(guard_syncstate))
	{

	}

	bool isRelay()
	{
		return state == LocalPlannerStateMachine_States::RELAY;
	}

	bool isReplanning()
	{
		return state == LocalPlannerStateMachine_States::REPLANNING;
	}

	bool isWaiting()
	{
		return state == LocalPlannerStateMachine_States::WAITING;
	}

	virtual void stepsignalprocess(std::shared_ptr<rei::AbstractSignalInterface> sig) override
	{
		using namespace trajectory_signals;
		switch (sig->getId())
		{
			case SignalReplanningTrajectory::signal_id:
			{
				reactSig_ReplanningTrajectory(sig);
				break;
			}
			case SignalNoObstacleDetected::signal_id:
			{
				reactSig_SignalNoObstacleDetected(sig);
				break;
			}
			case SignalLastWaypointReached::signal_id:
			{
				reactSig_LastWaypointReached(sig);
				break;
			}
			case SignalNewGlobalPlan::signal_id:
			{
				reactSig_NewGlobalPlan(sig);
				break;
			}
		}
	}

	virtual void reactSig_ReplanningTrajectory(std::shared_ptr<rei::AbstractSignalInterface> sig) override
	{
		// TODO:
		switch(state)
		{
			case LocalPlannerStateMachine_States::RELAY:
			{

				if (guard_def->guard_Relay2ReplanningState())
				{
					state = LocalPlannerStateMachine_States::REPLANNING;
					sig->effect();
				}
				break;
			}
		}
	}

	virtual void reactSig_SignalNoObstacleDetected(std::shared_ptr<rei::AbstractSignalInterface> sig) override
	{
		// TODO:
		switch(state)
		{
			case LocalPlannerStateMachine_States::REPLANNING:
			{
				if (guard_def->guard_Replanning2RelayState())
				{
					state = LocalPlannerStateMachine_States::RELAY;
					sig->effect();
				}
				break;
			}
		}
	}

	virtual void reactSig_LastWaypointReached(std::shared_ptr<rei::AbstractSignalInterface> sig) override
	{
		switch(state)
		{
			case LocalPlannerStateMachine_States::RELAY:
			{
				if (guard_def->guard_Relay2Waiting())
				{
					state = LocalPlannerStateMachine_States::WAITING;
					sig->effect();
				}
				break;
			}
		}
	}

	virtual void reactSig_NewGlobalPlan(std::shared_ptr<rei::AbstractSignalInterface> sig) override
	{
		switch(state)
		{
			case LocalPlannerStateMachine_States::WAITING:
			{
				if (guard_def->guard_Waiting2Relay())
				{
					state = LocalPlannerStateMachine_States::RELAY;
					sig->effect();
				}
				break;
			}
		}
	}
};

}


#endif /* INCLUDE_HOTARU_COMMON_STATE_MACHINE_TRAJECTORY_STATEMACHINE_HPP_ */
