/*
 * state_machine.hpp
 *
 *  Created on: Jan 30, 2020
 *      Author: kyberszittya
 */

#ifndef INCLUDE_REI_COMMON_STATE_MACHINE_SYNC_STATE_MACHINE_HPP_
#define INCLUDE_REI_COMMON_STATE_MACHINE_SYNC_STATE_MACHINE_HPP_



#include <rei_statemachine_library/abstract_statemachine_definitions.hpp>
#include "sync_signal.hpp"

namespace rei
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
protected:

public:
	virtual ~Interface_ReactSyncSignals() = 0;
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
	virtual ~Interface_GuardSyncStates() = 0;
	// Guard started state
	virtual bool guard_StartedState() = 0;
	virtual bool guard_WaitingState() = 0;
	virtual bool guard_Stopped() = 0;
};



class SyncStateMachine: public AbstractStateMachine<
		SyncStateMachine_States,
		Interface_GuardSyncStates>,
	public Interface_ReactSyncSignals
{
private:
protected:

	virtual void handle_start() override;

	virtual void handle_startError() override;

	//
	std::function<void(void)> cbAllStateMessageReceived;
	std::function<void(void)> cbTimeOut;
public:
	SyncStateMachine(
			std::shared_ptr<Interface_CommunicationGraphNotifier> graph_notifier,
			std::shared_ptr<Interface_GuardSyncStates> guard_syncstate);

	bool isWaiting() const
	{
		return state == SyncStateMachine_States::WAITING;
	}

	void setCbAllStateMessageReceived(std::function<void(void)> cb)
	{
		cbAllStateMessageReceived = cb;
	}

	void setCbTimeOut(std::function<void(void)> cb)
	{
		cbTimeOut = cb;
	}

	bool isStarted() const
	{
		return state == SyncStateMachine_States::STARTED;
	}

	bool isStopped() const
	{
		return state == SyncStateMachine_States::STOPPED;
	}

	virtual void stepsignalprocess(std::shared_ptr<AbstractSignalInterface> sig) override;

	virtual void reactSig_AllStateMessagesReceived(std::shared_ptr<AbstractSignalInterface> sig) override;
	virtual void reactSig_MessageTimeout(std::shared_ptr<AbstractSignalInterface> sig) override;
	virtual void reactSig_TerminationRequest(std::shared_ptr<AbstractSignalInterface> sig) override;


};

}

#endif /* INCLUDE_HOTARU_COMMON_STATE_MACHINE_SYNC_STATE_MACHINE_HPP_ */
