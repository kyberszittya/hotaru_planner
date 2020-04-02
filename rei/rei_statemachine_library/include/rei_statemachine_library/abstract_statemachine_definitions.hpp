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
#include <sstream>
#include <mutex>
#include <functional>

#include "abstract_signal_definitions.hpp"

namespace rei
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
 *
 */
class StateMachineEmptySignalBuffer: public std::exception
{
private:
	std::string message;
	std::string component;
public:
	StateMachineEmptySignalBuffer(
			std::string component,
			std::string failure): std::exception()
	{

	}

	virtual const char* what()
	{
		std::stringstream ss;
		ss << "Empty signal buffer (";
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
	virtual ~Interface_CommunicationGraphNotifier()  = 0;
	// Notify on signal
	virtual void notifyCommunicationGraph(std::shared_ptr<AbstractSignalInterface> sig) = 0;
};

/**
 * Abstract definition for guard components
 */
class Abstract_StateMachineGuard
{
public:
	virtual ~Abstract_StateMachineGuard() = 0;
	//
};

class StateMachineRunner
{
protected:
	std::queue<std::shared_ptr<AbstractSignalInterface>> signal_buffer;
	std::shared_ptr<Interface_CommunicationGraphNotifier> comm_graph_notifier;

	virtual ~StateMachineRunner();
	virtual bool _isAllocated() const = 0;
	virtual void handle_start() = 0;
	virtual void handle_startError() = 0;
	std::mutex mtx_step;
public:
	StateMachineRunner(std::shared_ptr<Interface_CommunicationGraphNotifier> graph_notifier):
		comm_graph_notifier(std::move(graph_notifier)){}

	bool start();

	void propagateSignal(std::shared_ptr<AbstractSignalInterface> sig);

	virtual void stepsignalprocess(std::shared_ptr<AbstractSignalInterface> sig) = 0;

	void stepstatemachine();
};


template<class StateDef, class GuardDef> class AbstractStateMachine: public StateMachineRunner
{
private:

protected:
	StateDef state;
	std::shared_ptr<GuardDef> guard_def;
	AbstractStateMachine(StateDef state,
			std::shared_ptr<Interface_CommunicationGraphNotifier> graph_notifier,
			std::shared_ptr<GuardDef> guard_def):
				StateMachineRunner(graph_notifier),
				state(state),
				guard_def(guard_def){}


	virtual bool _isAllocated() const
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
	// Add functions to handle at start
	std::function<void(void)> start_f;
public:

	const StateDef getState()
	{
		return state;
	}

	void setStartFunction(std::function<void(void)> f)
	{
		start_f = f;
	}
};

}

#endif /* INCLUDE_HOTARU_COMMON_STATE_MACHINE_ABSTRACT_STATEMACHINE_DEFINITIONS_HPP_ */
