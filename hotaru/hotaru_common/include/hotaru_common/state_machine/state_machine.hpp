/*
 * state_machine.hpp
 *
 *  Created on: Jan 30, 2020
 *      Author: kyberszittya
 */

#ifndef INCLUDE_HOTARU_COMMON_STATE_MACHINE_STATE_MACHINE_HPP_
#define INCLUDE_HOTARU_COMMON_STATE_MACHINE_STATE_MACHINE_HPP_




#include "abstract_statemachine_definitions.hpp"
#include "sync_signal.hpp"

namespace hotaru
{



enum class SyncStateMachine_States {
	PSEUDO_START,
	WAITING,
	STARTED,
	STOPPED,
	PSEUDO_EXIT
};

/**
 * Interface to define reactions to sync signals
 */
class Interface_ReactSyncSignals
{
public:
	// React to all state messages received signal
	virtual void reactSig_AllStateMessagesReceived(std::shared_ptr<AbstractSignalInterface> sig) = 0;
	// React to timeout message
	virtual void reactSig_MessageTimeout(std::shared_ptr<AbstractSignalInterface> sig) = 0;
	// React to termination request
	virtual void reactSig_TerminationRequest(std::shared_ptr<AbstractSignalInterface> sig) = 0;

};

/**
 * Interface to guard states of sync state machine
 */
class Interface_GuardSyncStates: public Abstract_StateMachineGuard
{
public:
	// Guard started state
	virtual bool guard_StartedState() = 0;
	virtual bool guard_WaitingState() = 0;
	virtual bool guard_Stopped() = 0;
};



class SyncStateMachine: public AbstractStateMachine<SyncStateMachine_States,
		Interface_GuardSyncStates>,
	public Interface_ReactSyncSignals
{
private:
protected:

public:
	SyncStateMachine(
			std::unique_ptr<Interface_CommunicationGraphNotifier> graph_notifier,
			std::unique_ptr<Interface_GuardSyncStates> guard_syncstate):
		AbstractStateMachine(SyncStateMachine_States::PSEUDO_START,
				std::move(graph_notifier),
				std::move(guard_syncstate))
	{

	}

	bool isWaiting()
	{
		return state == SyncStateMachine_States::WAITING;
	}

	bool isStarted()
	{
		return state == SyncStateMachine_States::STARTED;
	}

	bool isStopped()
	{
		return state == SyncStateMachine_States::STOPPED;
	}





	virtual void stepstatemachine() override
	{
		std::shared_ptr<AbstractSignalInterface> sig = signal_buffer.front();
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
		signal_buffer.pop();
	}

	void start()
	{
		if (isAllocated())
		{
			switch(state)
			{
			case SyncStateMachine_States::PSEUDO_START:
			{
				state = SyncStateMachine_States::WAITING;

				break;
			}
			}
		}
		else
		{
			throw StateMachineStartFailure("sync_state_machine","allocation error");
		}
	}

	virtual void reactSig_AllStateMessagesReceived(std::shared_ptr<AbstractSignalInterface> sig) override
	{
		switch(state)
		{
			case SyncStateMachine_States::WAITING:
			{
				if (guard_def->guard_StartedState())
				{
					state = SyncStateMachine_States::STARTED;
					comm_graph_notifier->notifyCommunicationGraph(sig);
				}
				break;
			}
		}
	}

	virtual void reactSig_MessageTimeout(std::shared_ptr<AbstractSignalInterface> sig) override
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

	virtual void reactSig_TerminationRequest(std::shared_ptr<AbstractSignalInterface> sig) override
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
};

}

#endif /* INCLUDE_HOTARU_COMMON_STATE_MACHINE_STATE_MACHINE_HPP_ */
