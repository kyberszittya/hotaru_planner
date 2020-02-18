/*
 * test_common.hpp
 *
 *  Created on: Feb 10, 2020
 *      Author: kyberszittya
 */

#ifndef TEST_REI_STATE_MACHINE_LIBRARY_TEST_COMMON_HPP_
#define TEST_REI_STATE_MACHINE_LIBRARY_TEST_COMMON_HPP_

#include <gtest/gtest.h>

#include <rei_statemachine_library/abstract_statemachine_definitions.hpp>
#include <rei_statemachine_library/portsync_state_machine/sync_signal.hpp>

#include <memory>
#include <iostream>
#include <list>

#include <rei_statemachine_library/portsync_state_machine/sync_state_machine.hpp>

namespace rei
{


class DummyCommunicationGraphNotifier: public Interface_CommunicationGraphNotifier
{
private:
	std::shared_ptr<std::list<unsigned long>> sig_trace;
public:
	DummyCommunicationGraphNotifier(): Interface_CommunicationGraphNotifier(),
		sig_trace(new std::list<unsigned long>())
	{}

	virtual void notifyCommunicationGraph(std::shared_ptr<AbstractSignalInterface> sig) override
	{
		sig_trace->push_back(sig->getId());
		std::cout << "NOTIFY with sig: " << sig->getId() << '\n';
	}

	std::shared_ptr<std::list<unsigned long>> getSignalTrace()
	{
		return sig_trace;
	}

	~DummyCommunicationGraphNotifier()
	{

	}

};


namespace test_sync_state_machine
{

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
//

template<typename T> bool verifyTrace(std::vector<T>& required_trace_list,
		std::shared_ptr<DummyCommunicationGraphNotifier> dummy_comm_graph)
{
	bool res = true;
	int i = 0;
	for (const auto&v: (*dummy_comm_graph->getSignalTrace()))
	{
		res &= required_trace_list[i] == v;
		i++;
	}
	return res;
}

}
#endif /* TEST_TEST_COMMON_HPP_ */
