/*
 * test_common.hpp
 *
 *  Created on: Feb 10, 2020
 *      Author: kyberszittya
 */

#ifndef TEST_TEST_COMMON_HPP_
#define TEST_TEST_COMMON_HPP_

#include <gtest/gtest.h>

#include <hotaru_common/state_machine/state_machine.hpp>
#include <hotaru_common/state_machine/sync_signal.hpp>
#include <memory>

#include <iostream>

namespace hotaru
{

class DummyCommunicationGraphNotifier: public Interface_CommunicationGraphNotifier
{
public:
	DummyCommunicationGraphNotifier(): Interface_CommunicationGraphNotifier(){}

	virtual void notifyCommunicationGraph(std::shared_ptr<AbstractSignalInterface> sig) override
	{
		std::cout << "NOTIFY with sig: " << sig->getId() << '\n';
	}

};

class DummySyncStateGuard: public Interface_GuardSyncStates
{
public:
	virtual bool guard_StartedState() override
	{
		return true;
	}

	virtual bool guard_WaitingState() override
	{
		return true;
	}

	virtual bool guard_Stopped() override
	{
		return true;
	}
};

class DummySyncStateDenyGuard: public Interface_GuardSyncStates
{
public:
	virtual bool guard_StartedState() override
	{
		return false;
	}

	virtual bool guard_WaitingState() override
	{
		return false;
	}

	virtual bool guard_Stopped() override
	{
		return false;
	}
};

}
#endif /* TEST_TEST_COMMON_HPP_ */
