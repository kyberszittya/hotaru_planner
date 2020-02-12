/*
 * abstract_definitions.hpp
 *
 *  Created on: Jan 30, 2020
 *      Author: kyberszittya
 */

#ifndef INCLUDE_HOTARU_COMMON_STATE_MACHINE_ABSTRACT_STATEMACHINE_DEFINITIONS_HPP_
#define INCLUDE_HOTARU_COMMON_STATE_MACHINE_ABSTRACT_STATEMACHINE_DEFINITIONS_HPP_

#include <queue>
#include <memory>
#include <exception>

#include "abstract_signal_definitions.hpp"

namespace hotaru
{

/**
 * State machine start failure
 */
class StateMachineStartFailure: public std::exception
{
private:
	std::string message;
	std::string component;
public:
	StateMachineStartFailure(
			std::string component,
			std::string failure): std::exception()
	{

	}

	virtual const char* what()
	{
		std::stringstream ss;
		ss << "Unable to start state machine (";
		ss << component.c_str();
		ss << "): ";
		ss << message.c_str();
		return ss.str().c_str();
	}
};

/**
 * Interface to notify external communication graph
 */
class Interface_CommunicationGraphNotifier
{
public:
	// Notify on signal
	virtual void notifyCommunicationGraph(std::shared_ptr<AbstractSignalInterface> sig) = 0;
};

/**
 * Abstract definition for guard components
 */
class Abstract_StateMachineGuard
{
public:
	//
};


template<class StateDef, class GuardDef> class AbstractStateMachine
{
private:

protected:
	StateDef state;
	std::queue<std::shared_ptr<AbstractSignalInterface>> signal_buffer;
	std::unique_ptr<Interface_CommunicationGraphNotifier> comm_graph_notifier;
	std::unique_ptr<GuardDef> guard_def;
	AbstractStateMachine(StateDef state,
			std::unique_ptr<Interface_CommunicationGraphNotifier> graph_notifier,
			std::unique_ptr<GuardDef> guard_def):
				state(state),
				comm_graph_notifier(std::move(graph_notifier)),
				guard_def(std::move(guard_def)){}

	bool isAllocated() const
	{
		if (guard_def == nullptr)
		{
			return false;
		}
		if (comm_graph_notifier == nullptr)
		{
			return false;
		}
		return true;
	}
public:

	virtual void stepstatemachine() = 0;

	void propagateSignal(std::shared_ptr<AbstractSignalInterface> sig)
	{
		signal_buffer.push(sig);
	}

	const StateDef getState()
	{
		return state;
	}


};

}

#endif /* INCLUDE_HOTARU_COMMON_STATE_MACHINE_ABSTRACT_STATEMACHINE_DEFINITIONS_HPP_ */
