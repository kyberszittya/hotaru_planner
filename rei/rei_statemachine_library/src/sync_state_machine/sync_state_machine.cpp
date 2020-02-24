/*
 * sync_state_machine.cpp
 *
 *  Created on: Feb 18, 2020
 *      Author: kyberszittya
 */

#include <rei_statemachine_library/portsync_state_machine/sync_state_machine.hpp>

namespace rei
{

void SyncStateMachine::handle_start()
{
	switch(state)
	{
		case SyncStateMachine_States::PSEUDO_START:
		{
			state = SyncStateMachine_States::WAITING;
			start_f();
			break;
		}
	}
}

void SyncStateMachine::handle_startError()
{
	throw  StateMachineStartFailure("sync_state_machine", "start");
}

SyncStateMachine::SyncStateMachine(
			std::shared_ptr<Interface_CommunicationGraphNotifier> graph_notifier,
			std::unique_ptr<Interface_GuardSyncStates> guard_syncstate):
		AbstractStateMachine(SyncStateMachine_States::PSEUDO_START,
						graph_notifier,
						std::move(guard_syncstate))
{

}


/**
 * OVERRIDE state machine functions
 */
void SyncStateMachine::stepsignalprocess(std::shared_ptr<AbstractSignalInterface> sig)
{
	using namespace sync_signals;
	switch (sig->getId())
	{
	// Step to started, if all messages received
	case SignalAllStateMessagesReceived::signal_id:
		{
			reactSig_AllStateMessagesReceived(sig);
			break;
		}
	case SignalMessageTimeOut::signal_id:
		{
			reactSig_MessageTimeout(sig);
			break;
		}
	case SignalTerminationRequest::signal_id:
		{
			reactSig_TerminationRequest(sig);
			break;
		}
	}
}

void SyncStateMachine::reactSig_AllStateMessagesReceived(std::shared_ptr<AbstractSignalInterface> sig)
{
	switch(state)
	{
		case SyncStateMachine_States::WAITING:
		{
			if (guard_def->guard_StartedState())
			{
				state = SyncStateMachine_States::STARTED;
				cbAllStateMessageReceived();
				comm_graph_notifier->notifyCommunicationGraph(sig);
			}
			break;
		}
	}
}

void SyncStateMachine::reactSig_MessageTimeout(std::shared_ptr<AbstractSignalInterface> sig)
{
	switch(state)
	{
		case SyncStateMachine_States::STARTED:
		{
			if (guard_def->guard_WaitingState())
			{
				state = SyncStateMachine_States::WAITING;
				comm_graph_notifier->notifyCommunicationGraph(sig);
			}
			break;
		}
	}
}

void SyncStateMachine::reactSig_TerminationRequest(std::shared_ptr<AbstractSignalInterface> sig)
{
	switch(state)
	{
		case SyncStateMachine_States::STARTED:
		case SyncStateMachine_States::WAITING:
		{
			if (guard_def->guard_Stopped())
			{
				state = SyncStateMachine_States::STOPPED;
				comm_graph_notifier->notifyCommunicationGraph(sig);
			}
			break;
		}
	}
}

// Abstract destructors
Interface_GuardSyncStates::~Interface_GuardSyncStates(){}
Interface_ReactSyncSignals::~Interface_ReactSyncSignals(){}

}





