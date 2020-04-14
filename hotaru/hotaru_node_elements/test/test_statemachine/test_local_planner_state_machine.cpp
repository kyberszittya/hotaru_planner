/*
 * test_local_planner_state_machine.cpp
 *
 *  Created on: Feb 17, 2020
 *      Author: kyberszittya
 */

#include "test_common.hpp"
#include <rei_statemachine_library/testing/test_common.hpp>

TEST(LocalPlannerStateMachine, LocalPlannerStateMachineSingleReplanningTrace)
{
	using namespace hotaru;
	using namespace hotaru::trajectory_signals;
	std::shared_ptr<rei::DummyCommunicationGraphNotifier> dummy_comm_graph(new rei::DummyCommunicationGraphNotifier());
	std::unique_ptr<test_local_planner_state_machine::DummyLocalPlannerStateMachineGuard> dummy_guard(
			new test_local_planner_state_machine::DummyLocalPlannerStateMachineGuard());
	LocalPlannerStateMachine loc_sm(dummy_comm_graph, std::move(dummy_guard));
	ASSERT_TRUE(loc_sm.start());
	ASSERT_TRUE(loc_sm.isRelay());
	std::shared_ptr<rei::AbstractSignalInterface> sig_replanning_1(new SignalReplanningTrajectory(0));
	loc_sm.propagateSignal(sig_replanning_1);
	loc_sm.stepstatemachine();
	ASSERT_TRUE(loc_sm.isReplanning());
	std::shared_ptr<rei::AbstractSignalInterface> sig_noobstacle_1(new SignalNoObstacleDetected(1));
	loc_sm.propagateSignal(sig_noobstacle_1);
	loc_sm.stepstatemachine();
	ASSERT_TRUE(loc_sm.isRelay());
	std::vector<unsigned long> trace({32, 33});
	ASSERT_TRUE(rei::verifyTrace<unsigned long>(trace, dummy_comm_graph));
}


TEST(LocalPlannerStateMachine, LocalPlannerStateMachineSingleTrace)
{
	using namespace hotaru;
	using namespace hotaru::trajectory_signals;
	std::shared_ptr<rei::DummyCommunicationGraphNotifier> dummy_comm_graph(new rei::DummyCommunicationGraphNotifier());
	std::unique_ptr<test_local_planner_state_machine::DummyLocalPlannerStateMachineGuard> dummy_guard(
			new test_local_planner_state_machine::DummyLocalPlannerStateMachineGuard());
	LocalPlannerStateMachine loc_sm(dummy_comm_graph, std::move(dummy_guard));
	ASSERT_TRUE(loc_sm.start());
	ASSERT_TRUE(loc_sm.isRelay());
	std::shared_ptr<rei::AbstractSignalInterface> sig_replanning_1(new SignalReplanningTrajectory(0));
	loc_sm.propagateSignal(sig_replanning_1);
	loc_sm.stepstatemachine();
	ASSERT_TRUE(loc_sm.isReplanning());
	std::shared_ptr<rei::AbstractSignalInterface> sig_noobstacle_1(new SignalNoObstacleDetected(1));
	loc_sm.propagateSignal(sig_noobstacle_1);
	loc_sm.stepstatemachine();
	ASSERT_TRUE(loc_sm.isRelay());
	// Stop, global plan reached
	std::shared_ptr<rei::AbstractSignalInterface> sig_lastwaypoint_reached(new SignalLastWaypointReached(2));
	loc_sm.propagateSignal(sig_lastwaypoint_reached);
	loc_sm.stepstatemachine();
	ASSERT_TRUE(loc_sm.isWaiting());
	std::vector<unsigned long> trace({32, 33, 34});
	ASSERT_TRUE(rei::verifyTrace<unsigned long>(trace, dummy_comm_graph));
}


TEST(LocalPlannerStateMachine, LocalPlannerReceiveNewGlobalPlan)
{
	using namespace hotaru;
	using namespace hotaru::trajectory_signals;
	std::shared_ptr<rei::DummyCommunicationGraphNotifier> dummy_comm_graph(new rei::DummyCommunicationGraphNotifier());
	std::unique_ptr<test_local_planner_state_machine::DummyLocalPlannerStateMachineGuard> dummy_guard(
			new test_local_planner_state_machine::DummyLocalPlannerStateMachineGuard());
	LocalPlannerStateMachine loc_sm(dummy_comm_graph, std::move(dummy_guard));
	ASSERT_TRUE(loc_sm.start());
	ASSERT_TRUE(loc_sm.isRelay());
	std::shared_ptr<rei::AbstractSignalInterface> sig_replanning_1(new SignalReplanningTrajectory(0));
	loc_sm.propagateSignal(sig_replanning_1);
	loc_sm.stepstatemachine();
	ASSERT_TRUE(loc_sm.isReplanning());
	std::shared_ptr<rei::AbstractSignalInterface> sig_noobstacle_1(new SignalNoObstacleDetected(1));
	loc_sm.propagateSignal(sig_noobstacle_1);
	loc_sm.stepstatemachine();
	// Stop, global plan reached
	std::shared_ptr<rei::AbstractSignalInterface> sig_lastwaypoint_reached(new SignalLastWaypointReached(2));
	loc_sm.propagateSignal(sig_lastwaypoint_reached);
	loc_sm.stepstatemachine();
	ASSERT_TRUE(loc_sm.isWaiting());
	// Receive new global plan
	std::shared_ptr<rei::AbstractSignalInterface> sig_newglobalplan(new SignalNewGlobalPlan(3));
	loc_sm.propagateSignal(sig_newglobalplan);
	loc_sm.stepstatemachine();
	ASSERT_TRUE(loc_sm.isRelay());
	std::vector<unsigned long> trace({32, 33, 34, 35});
	ASSERT_TRUE(rei::verifyTrace<unsigned long>(trace, dummy_comm_graph));
}

TEST(LocalPlannerStateMachine, LocalPlannerReceiveNewGlobalPlanReplang)
{
	using namespace hotaru;
	using namespace hotaru::trajectory_signals;
	std::shared_ptr<rei::DummyCommunicationGraphNotifier> dummy_comm_graph(new rei::DummyCommunicationGraphNotifier());
	std::unique_ptr<test_local_planner_state_machine::DummyLocalPlannerStateMachineGuard> dummy_guard(
			new test_local_planner_state_machine::DummyLocalPlannerStateMachineGuard());
	LocalPlannerStateMachine loc_sm(dummy_comm_graph, std::move(dummy_guard));
	ASSERT_TRUE(loc_sm.start());
	ASSERT_TRUE(loc_sm.isRelay());
	std::shared_ptr<rei::AbstractSignalInterface> sig_replanning_1(new SignalReplanningTrajectory(0));
	loc_sm.propagateSignal(sig_replanning_1);
	loc_sm.stepstatemachine();
	ASSERT_TRUE(loc_sm.isReplanning());
	std::shared_ptr<rei::AbstractSignalInterface> sig_noobstacle_1(new SignalNoObstacleDetected(1));
	loc_sm.propagateSignal(sig_noobstacle_1);
	loc_sm.stepstatemachine();
	// Stop, global plan reached
	std::shared_ptr<rei::AbstractSignalInterface> sig_lastwaypoint_reached(new SignalLastWaypointReached(2));
	loc_sm.propagateSignal(sig_lastwaypoint_reached);
	loc_sm.stepstatemachine();
	ASSERT_TRUE(loc_sm.isWaiting());
	// Receive new global plan
	std::shared_ptr<rei::AbstractSignalInterface> sig_newglobalplan(new SignalNewGlobalPlan(3));
	loc_sm.propagateSignal(sig_newglobalplan);
	loc_sm.stepstatemachine();
	ASSERT_TRUE(loc_sm.isRelay());
	// Replan
	std::shared_ptr<rei::AbstractSignalInterface> sig_replanning_2(new SignalReplanningTrajectory(5));
	loc_sm.propagateSignal(sig_replanning_2);
	loc_sm.stepstatemachine();
	ASSERT_TRUE(loc_sm.isReplanning());
	std::vector<unsigned long> trace({32, 33, 34, 35, 32});
	ASSERT_TRUE(rei::verifyTrace<unsigned long>(trace, dummy_comm_graph));
}

int main(int argc, char **argv){
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
