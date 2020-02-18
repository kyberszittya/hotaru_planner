/*
 * test_sync_state_machine.cpp
 *
 *  Created on: Jan 31, 2020
 *      Author: kyberszittya
 */


#include <rei_statemachine_library/testing/test_common.hpp>

TEST(SyncMachineBasicTest, BasicTest)
{
	using namespace rei;
	std::unique_ptr<test_sync_state_machine::DummySyncStateGuard> dummy_syncstate(
			new test_sync_state_machine::DummySyncStateGuard());
	std::shared_ptr<DummyCommunicationGraphNotifier> dummy_comm_graph(new DummyCommunicationGraphNotifier());
	SyncStateMachine sm(dummy_comm_graph,
			std::move(dummy_syncstate));
	sm.start();
	using namespace rei::sync_signals;
	ASSERT_TRUE(sm.isWaiting());
	std::shared_ptr<AbstractSignalInterface> sig_sync(new SignalAllStateMessagesReceived(3));
	sm.propagateSignal(sig_sync);
	sm.stepstatemachine();
	ASSERT_TRUE(sm.isStarted());
	std::shared_ptr<AbstractSignalInterface> sig_term(new SignalTerminationRequest(4));
	sm.propagateSignal(sig_term);
	sm.stepstatemachine();
	ASSERT_TRUE(sm.isStopped());
	std::vector<unsigned long> trace({1, 65535});
	ASSERT_TRUE(verifyTrace<unsigned long>(trace, dummy_comm_graph));
}

TEST(SyncMachineBasicTest, BasicTestDeny)
{
	using namespace rei;
	std::unique_ptr<test_sync_state_machine::DummySyncStateDenyGuard> dummy_syncstate(
			new test_sync_state_machine::DummySyncStateDenyGuard());
	std::shared_ptr<DummyCommunicationGraphNotifier> dummy_comm_graph(new DummyCommunicationGraphNotifier());
	SyncStateMachine sm(dummy_comm_graph,
			std::move(dummy_syncstate));
	sm.start();
	using namespace rei::sync_signals;
	ASSERT_TRUE(sm.isWaiting());
	std::shared_ptr<AbstractSignalInterface> sig_sync(new SignalAllStateMessagesReceived(3));
	sm.propagateSignal(sig_sync);
	sm.stepstatemachine();
	ASSERT_FALSE(sm.isStarted());
	std::shared_ptr<AbstractSignalInterface> sig_term(new SignalTerminationRequest(4));
	sm.propagateSignal(sig_term);
	sm.stepstatemachine();
	ASSERT_FALSE(sm.isStopped());
	std::vector<unsigned long> trace({1, 2, 1, 65535});
	ASSERT_EQ(dummy_comm_graph->getSignalTrace()->size(), 0);
}

TEST(SyncMachineBasicTest, BasicTestTmeout)
{
	using namespace rei;
	std::unique_ptr<test_sync_state_machine::DummySyncStateGuard> dummy_syncstate(
			new test_sync_state_machine::DummySyncStateGuard());
	std::shared_ptr<DummyCommunicationGraphNotifier> dummy_comm_graph(new DummyCommunicationGraphNotifier());
	SyncStateMachine sm(
			dummy_comm_graph,
			std::move(dummy_syncstate));
	ASSERT_TRUE(sm.start());

	using namespace rei::sync_signals;
	ASSERT_TRUE(sm.isWaiting());
	std::shared_ptr<AbstractSignalInterface> sig_sync(new SignalAllStateMessagesReceived(1));
	sm.propagateSignal(sig_sync);
	sm.stepstatemachine();
	ASSERT_TRUE(sm.isStarted());
	std::shared_ptr<AbstractSignalInterface> sig_timeout(new SignalMessageTimeOut(2));
	sm.propagateSignal(sig_timeout);
	sm.stepstatemachine();
	ASSERT_TRUE(sm.isWaiting());
	// Recover from timeout
	std::shared_ptr<AbstractSignalInterface> sig_sync2(new SignalAllStateMessagesReceived(3));
	sm.propagateSignal(sig_sync);
	sm.stepstatemachine();
	ASSERT_TRUE(sm.isStarted());
	// Terminate
	std::shared_ptr<AbstractSignalInterface> sig_term(new SignalTerminationRequest(4));
	sm.propagateSignal(sig_term);
	sm.stepstatemachine();
	ASSERT_TRUE(sm.isStopped());
	std::vector<unsigned long> trace({1, 2, 1, 65535});
	ASSERT_TRUE(verifyTrace<unsigned long>(trace, dummy_comm_graph));
}

int main(int argc, char **argv){
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
