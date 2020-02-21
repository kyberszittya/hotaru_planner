/*
 * test_abstract_definitions.cpp
 *
 *  Created on: Feb 19, 2020
 *      Author: kyberszittya
 */


#include <rei_statemachine_library/testing/test_common.hpp>
#include <rei_statemachine_library/abstract_statemachine_definitions.hpp>

enum class DummyStates { PSEUDO_START, PSEUDO_END };

class DummyGuard
{

};

class DummyStateMachine: public rei::AbstractStateMachine<DummyStates, DummyGuard>
{
protected:
	virtual void handle_start() override{}
	virtual void handle_startError() override{}

public:
	DummyStateMachine(std::unique_ptr<DummyGuard> guard,
			std::shared_ptr<rei::Interface_CommunicationGraphNotifier> comm_graph):
				AbstractStateMachine(DummyStates::PSEUDO_START,
						comm_graph,
						std::move(guard))
				{

				}

	virtual void stepsignalprocess(std::shared_ptr<rei::AbstractSignalInterface> sig)
	{

	}
};

class DummyStateMachineErrorException: public rei::AbstractStateMachine<DummyStates, DummyGuard>
{
protected:
	virtual void handle_start() override{}
	virtual void handle_startError() override
	{
		throw rei::StateMachineStartFailure("dummy_state_machine", "");
	}

public:
	DummyStateMachineErrorException(std::unique_ptr<DummyGuard> guard,
			std::shared_ptr<rei::Interface_CommunicationGraphNotifier> comm_graph):
				AbstractStateMachine(DummyStates::PSEUDO_START,
						comm_graph,
						std::move(guard))
				{

				}

	virtual void stepsignalprocess(std::shared_ptr<rei::AbstractSignalInterface> sig)
	{

	}
};


TEST(AbstractStateMachine, NoAllocationTest)
{
	using namespace rei;
	std::unique_ptr<DummyGuard> dummy_guardstate;
	std::shared_ptr<DummyCommunicationGraphNotifier> dummy_comm_graph;
	DummyStateMachine dummy_state_machine(std::move(dummy_guardstate), dummy_comm_graph);
	ASSERT_FALSE(dummy_state_machine.start());
}

TEST(AbstractStateMachine, NoAllocationTestNoGuardState)
{
	using namespace rei;
	std::unique_ptr<DummyGuard> dummy_guardstate(new DummyGuard());
	std::shared_ptr<DummyCommunicationGraphNotifier> dummy_comm_graph;
	DummyStateMachine dummy_state_machine(std::move(dummy_guardstate), dummy_comm_graph);
	ASSERT_FALSE(dummy_state_machine.start());
}

TEST(AbstractStateMachine, NoAllocationTestCommGraph)
{
	using namespace rei;
	std::unique_ptr<DummyGuard> dummy_guardstate;
	std::shared_ptr<DummyCommunicationGraphNotifier> dummy_comm_graph(new DummyCommunicationGraphNotifier());
	DummyStateMachine dummy_state_machine(std::move(dummy_guardstate), dummy_comm_graph);
	ASSERT_FALSE(dummy_state_machine.start());
}

TEST(AbstractStateMachine, NoAllocationTestExceptionThrown)
{
	using namespace rei;
	std::unique_ptr<DummyGuard> dummy_guardstate;
	std::shared_ptr<DummyCommunicationGraphNotifier> dummy_comm_graph(new DummyCommunicationGraphNotifier());
	DummyStateMachineErrorException dummy_state_machine(std::move(dummy_guardstate), dummy_comm_graph);
	ASSERT_THROW(dummy_state_machine.start(), StateMachineStartFailure);
	bool err = true;
	try
	{
		dummy_state_machine.start();
	}
	catch(StateMachineStartFailure& e)
	{
		err = false;
		std::cerr << e.what() << '\n';

	}
	ASSERT_FALSE(err);
}

TEST(AbstractStateMachine, AllocationFine)
{
	using namespace rei;
	std::unique_ptr<DummyGuard> dummy_guardstate(new DummyGuard());
	std::shared_ptr<DummyCommunicationGraphNotifier> dummy_comm_graph(new DummyCommunicationGraphNotifier());
	DummyStateMachine dummy_state_machine(std::move(dummy_guardstate), dummy_comm_graph);
	ASSERT_TRUE(dummy_state_machine.start());
	ASSERT_THROW(dummy_state_machine.stepstatemachine(), StateMachineEmptySignalBuffer);
}

TEST(AbstractStateMachine, AllocationFineWithException)
{
	using namespace rei;
	std::unique_ptr<DummyGuard> dummy_guardstate(new DummyGuard());
	std::shared_ptr<DummyCommunicationGraphNotifier> dummy_comm_graph(new DummyCommunicationGraphNotifier());
	DummyStateMachineErrorException dummy_state_machine(std::move(dummy_guardstate), dummy_comm_graph);
	ASSERT_TRUE(dummy_state_machine.start());
	ASSERT_THROW(dummy_state_machine.stepstatemachine(), StateMachineEmptySignalBuffer);
	bool iserror = false;
	try
	{
		dummy_state_machine.stepstatemachine();
	}
	catch(StateMachineEmptySignalBuffer& e)
	{
		iserror = true;
		std::cerr << e.what() << '\n';
	}
	ASSERT_TRUE(iserror);
}

int main(int argc, char **argv){
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
