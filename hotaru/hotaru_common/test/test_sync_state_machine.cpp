/*
 * test_sync_state_machine.cpp
 *
 *  Created on: Jan 31, 2020
 *      Author: kyberszittya
 */


#include "test_common.hpp"

TEST(SyncMachineBasicTest, BasicTest)
{
	std::unique_ptr<hotaru::DummySyncStateGuard> dummy_syncstate(new hotaru::DummySyncStateGuard());
	std::unique_ptr<hotaru::DummyCommunicationGraphNotifier> dummy_comm_graph(new hotaru::DummyCommunicationGraphNotifier());
	hotaru::SyncStateMachine sm(std::move(dummy_comm_graph),
			std::move(dummy_syncstate));
	sm.start();
	using namespace hotaru;
	using namespace hotaru::sync_signals;
	ASSERT_TRUE(sm.isWaiting());
	std::shared_ptr<AbstractSignalInterface> sig_sync(new SignalAllStateMessagesReceived(3));
	sm.propagateSignal(sig_sync);
	sm.stepstatemachine();
	ASSERT_TRUE(sm.isStarted());
	std::shared_ptr<AbstractSignalInterface> sig_term(new SignalTerminationRequest(4));
	sm.propagateSignal(sig_term);
	sm.stepstatemachine();
	ASSERT_TRUE(sm.isStopped());
}

TEST(SyncMachineBasicTest, BasicTestDeny)
{
	std::unique_ptr<hotaru::DummySyncStateDenyGuard> dummy_syncstate(new hotaru::DummySyncStateDenyGuard());
	std::unique_ptr<hotaru::DummyCommunicationGraphNotifier> dummy_comm_graph(new hotaru::DummyCommunicationGraphNotifier());
	hotaru::SyncStateMachine sm(std::move(dummy_comm_graph),
			std::move(dummy_syncstate));
	sm.start();
	using namespace hotaru;
	using namespace hotaru::sync_signals;
	ASSERT_TRUE(sm.isWaiting());
	std::shared_ptr<AbstractSignalInterface> sig_sync(new SignalAllStateMessagesReceived(3));
	sm.propagateSignal(sig_sync);
	sm.stepstatemachine();
	ASSERT_FALSE(sm.isStarted());
	std::shared_ptr<AbstractSignalInterface> sig_term(new SignalTerminationRequest(4));
	sm.propagateSignal(sig_term);
	sm.stepstatemachine();
	ASSERT_FALSE(sm.isStopped());
}

TEST(SyncMachineBasicTest, BasicTestTmeout)
{
	std::unique_ptr<hotaru::DummySyncStateGuard> dummy_syncstate(new hotaru::DummySyncStateGuard());
	std::unique_ptr<hotaru::DummyCommunicationGraphNotifier> dummy_comm_graph(new hotaru::DummyCommunicationGraphNotifier());
	hotaru::SyncStateMachine sm(
			std::move(dummy_comm_graph),
			std::move(dummy_syncstate));
	sm.start();
	using namespace hotaru;
	using namespace hotaru::sync_signals;
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
}

int main(int argc, char **argv){
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
