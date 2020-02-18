/*
 * abstract_state_machine.cpp
 *
 *  Created on: Feb 17, 2020
 *      Author: kyberszittya
 */

#include <rei_statemachine_library/abstract_signal_definitions.hpp>
#include <rei_statemachine_library/abstract_statemachine_definitions.hpp>


namespace rei
{

AbstractSignalInterface::~AbstractSignalInterface(){}
Abstract_StateMachineGuard::~Abstract_StateMachineGuard(){}

StateMachineRunner::~StateMachineRunner()
{
	comm_graph_notifier.reset();
}

bool StateMachineRunner::start()
{
	if (_isAllocated())
	{
		handle_start();
		return true;
	}
	else
	{
		handle_startError();
		return false;
	}
}

void StateMachineRunner::stepstatemachine()
{
	std::shared_ptr<AbstractSignalInterface> sig = signal_buffer.front();
	stepsignalprocess(sig);
	if (sig->isEffective())
	{
		comm_graph_notifier->notifyCommunicationGraph(sig);
	}
	signal_buffer.pop();
}

void StateMachineRunner::propagateSignal(std::shared_ptr<AbstractSignalInterface> sig)
{
	signal_buffer.push(sig);
}

Interface_CommunicationGraphNotifier::~Interface_CommunicationGraphNotifier() {}

}
