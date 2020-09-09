/*
 * test_basic_hybridstatemachine.cpp
 *
 *  Created on: Sep 3, 2020
 *      Author: kyberszittya
 */


#include <gtest/gtest.h>

#include <rei_construct_elements/hybrid_dynamic_system/rei_hybrid_state_machine.hpp>

#include "test_common.hpp"


/*
 * @test: Test basic construction of hybrid state machine without any transitions.
 */
TEST(TestHybridStateMachine, TestBasicConstruction)
{
	using namespace rei::node;
	HybridStateMachineFactory<unsigned long, DummyZeroClock>* factory = HybridStateMachineFactory<unsigned long, DummyZeroClock>::getInstance();
	std::shared_ptr<DummyZeroClock> sm_clock = std::make_shared<DummyZeroClock>();
	std::shared_ptr<HybridStateMachine<unsigned long, DummyZeroClock>> hy =
		factory->createHybridStateMachine("test_sm", DUMMY_DELTA_TIME, sm_clock);
	std::vector<std::string> loc_label = hy->getVertexLabels();
	ASSERT_EQ(loc_label[1], "PSEUDO_END");
	ASSERT_EQ(loc_label[0], "PSEUDO_START");


	ASSERT_EQ(hy->getCurrentLocation()->getLabel(), "PSEUDO_START");
	ASSERT_EQ(hy->getCurrentLocation()->getLocationNumber(), 0);

	// Check location mapping is correct!
	// REGR.REQ1: labels shall be always mapped to a correct location
	ASSERT_EQ(hy->getLocationByLabel("PSEUDO_START")->getLabel(), "PSEUDO_START");
	ASSERT_EQ(hy->getLocationByLabel("PSEUDO_START")->getLocationNumber(), 0);
	ASSERT_EQ(hy->getLocationByLabel("PSEUDO_END")->getLabel(), "PSEUDO_END");
	ASSERT_EQ(hy->getLocationByLabel("PSEUDO_END")->getLocationNumber(), 1);
	//
	ASSERT_EQ(hy->getNumberOfLocations(), 2);

}

/*
 * @test: Test construction of state machines with some additional states
 */
TEST(TestHybridStateMachine, TestAssignStatesBasicConstruction)
{
	using namespace rei::node;
	HybridStateMachineFactory<unsigned long, DummyZeroClock>* factory = HybridStateMachineFactory<unsigned long, DummyZeroClock>::getInstance();
	std::shared_ptr<DummyZeroClock> sm_clock = std::make_shared<DummyZeroClock>();
	std::shared_ptr<HybridStateMachine<unsigned long, DummyZeroClock>> hy =
		factory->createHybridStateMachine("test_sm", DUMMY_DELTA_TIME, sm_clock);
	factory->addLocations(*hy, {"WAITING", "ENABLED", "DISABLED"});
	std::vector<std::string> loc_label = hy->getVertexLabels();
	ASSERT_EQ(loc_label[0], "PSEUDO_START");
	ASSERT_EQ(hy->getLocationByLabel("PSEUDO_START")->getLocationNumber(), 0);
	ASSERT_EQ(loc_label[1], "PSEUDO_END");
	ASSERT_EQ(hy->getLocationByLabel("PSEUDO_END")->getLocationNumber(), 1);
	ASSERT_EQ(loc_label[2], "WAITING");
	ASSERT_EQ(hy->getLocationByLabel("WAITING")->getLocationNumber(), 2);
	ASSERT_EQ(loc_label[3], "ENABLED");
	ASSERT_EQ(hy->getLocationByLabel("ENABLED")->getLocationNumber(), 3);
	ASSERT_EQ(loc_label[4], "DISABLED");
	ASSERT_EQ(hy->getLocationByLabel("DISABLED")->getLocationNumber(), 4);
	ASSERT_EQ(hy->getCurrentLocation()->getLabel(), "PSEUDO_START");
	ASSERT_EQ(hy->getNumberOfLocations(), 5);
}

/*
 * @test: Test state machine of some states and with transitions (including incoming events).
 */
TEST(TestHybridStateMachine, TestAssignTransitionBasicConstruction)
{
	using namespace rei::node;
	HybridStateMachineFactory<unsigned long, DummyZeroClock>* factory =
			HybridStateMachineFactory<unsigned long, DummyZeroClock>::getInstance();
	std::shared_ptr<DummyZeroClock> sm_clock = std::make_shared<DummyZeroClock>();
	std::shared_ptr<HybridStateMachine<unsigned long, DummyZeroClock>> hy =
		factory->createHybridStateMachine("test_sm", DUMMY_DELTA_TIME, sm_clock);
	factory->addLocations(*hy, {"ON", "OFF"});
	std::vector<std::string> loc_label = hy->getVertexLabels();
	ASSERT_EQ(loc_label[0], "PSEUDO_START");
	ASSERT_EQ(loc_label[1], "PSEUDO_END");
	ASSERT_EQ(loc_label[2], "ON");
	ASSERT_EQ(loc_label[3], "OFF");
	ASSERT_EQ(hy->getNumberOfLocations(), 4);
	// Set transitions
	addTransitionsTestSM_OnOff(factory, hy);
	// Check step
	ASSERT_EQ(hy->getCurrentLocation()->getLabel(), "PSEUDO_START");
	// All transitions are enabled
	DiscreteEventPipeline<unsigned long, DummyZeroClock> pipeline_test;
	pipeline_test.setEventMapping(factory->getEventMapping());
	pipeline_test.addStateMachine(hy);
	pipeline_test.propagateEvent("StartEvent", 0);
	ASSERT_EQ(hy->step(), HybridStateStepResult::TRANSITED);
	ASSERT_EQ(hy->getCurrentLocation()->getLabel(), "ON");
	// Transition to another state
	pipeline_test.propagateEvent("StartEvent", 0);
	ASSERT_EQ(hy->step(), HybridStateStepResult::NO_TRANSITION);
	ASSERT_EQ(hy->getCurrentLocation()->getLabel(), "ON");
	pipeline_test.propagateEvent("foo", 0);
	ASSERT_EQ(hy->step(), HybridStateStepResult::TRANSITED);
	ASSERT_EQ(hy->getCurrentLocation()->getLabel(), "OFF");
	pipeline_test.propagateEvent("bar", 0);
	ASSERT_EQ(hy->step(), HybridStateStepResult::TRANSITED);
	ASSERT_EQ(hy->getCurrentLocation()->getLabel(), "ON");
	pipeline_test.propagateEvent("foo", 0);
	ASSERT_EQ(hy->step(), HybridStateStepResult::TRANSITED);
	ASSERT_EQ(hy->getCurrentLocation()->getLabel(), "OFF");
	pipeline_test.propagateEvent("baz", 0);
	ASSERT_EQ(hy->step(), HybridStateStepResult::TRANSITED);
	ASSERT_EQ(hy->getCurrentLocation()->getLabel(), "PSEUDO_END");
}

/*
 * @test: Test transiting state machine construction with initializer list
 */
TEST(TestHybridStateMachine, TestAssignTransitionInitilaizerListConstruction)
{
	using namespace rei::node;
	HybridStateMachineFactory<unsigned long, DummyZeroClock>* factory =
			HybridStateMachineFactory<unsigned long, DummyZeroClock>::getInstance();
	std::shared_ptr<DummyZeroClock> sm_clock = std::make_shared<DummyZeroClock>();
	std::shared_ptr<HybridStateMachine<unsigned long, DummyZeroClock>> hy =
		factory->createHybridStateMachine("test_sm", DUMMY_DELTA_TIME, sm_clock);
	factory->addLocations(*hy, {"ON", "OFF"});
	// Set transitions
	addTransitionsTestSM_OnOffTransitionList(factory, hy);
	std::vector<std::string> loc_label = hy->getVertexLabels();
	ASSERT_EQ(loc_label[0], "PSEUDO_START");
	ASSERT_EQ(loc_label[1], "PSEUDO_END");
	ASSERT_EQ(loc_label[2], "ON");
	ASSERT_EQ(loc_label[3], "OFF");
	ASSERT_EQ(hy->getNumberOfLocations(), 4);

	// Check step
	ASSERT_EQ(hy->getCurrentLocation()->getLabel(), "PSEUDO_START");
	// All transitions are enabled
	DiscreteEventPipeline<unsigned long, DummyZeroClock> pipeline_test;
	pipeline_test.setEventMapping(factory->getEventMapping());
	pipeline_test.addStateMachine(hy);
	pipeline_test.propagateEvent("StartEvent", 0);
	ASSERT_EQ(hy->step(), HybridStateStepResult::TRANSITED);
	ASSERT_EQ(hy->getCurrentLocation()->getLabel(), "ON");
	// Transition to another state
	pipeline_test.propagateEvent("StartEvent", 0);
	ASSERT_EQ(hy->step(), HybridStateStepResult::NO_TRANSITION);
	ASSERT_EQ(hy->getCurrentLocation()->getLabel(), "ON");
	pipeline_test.propagateEvent("foo", 0);
	ASSERT_EQ(hy->step(), HybridStateStepResult::TRANSITED);
	ASSERT_EQ(hy->getCurrentLocation()->getLabel(), "OFF");
	pipeline_test.propagateEvent("bar", 0);
	ASSERT_EQ(hy->step(), HybridStateStepResult::TRANSITED);
	ASSERT_EQ(hy->getCurrentLocation()->getLabel(), "ON");
	pipeline_test.propagateEvent("foo", 0);
	ASSERT_EQ(hy->step(), HybridStateStepResult::TRANSITED);
	ASSERT_EQ(hy->getCurrentLocation()->getLabel(), "OFF");
	pipeline_test.propagateEvent("baz", 0);
	ASSERT_EQ(hy->step(), HybridStateStepResult::TRANSITED);
	ASSERT_EQ(hy->getCurrentLocation()->getLabel(), "PSEUDO_END");
}


/*
 * @test: Test on multiple steps (ensure it's working even on empty event stack)
 * @req: REQ.HYSM.OP.1: the SM shall not transit on empty event stack
 */
TEST(TestHybridStateMachine, TestMultipleSteps)
{
	using namespace rei::node;
	HybridStateMachineFactory<unsigned long, DummyZeroClock>* factory =
			HybridStateMachineFactory<unsigned long, DummyZeroClock>::getInstance();
	std::shared_ptr<DummyZeroClock> sm_clock = std::make_shared<DummyZeroClock>();
	std::shared_ptr<HybridStateMachine<unsigned long, DummyZeroClock>> hy =
		factory->createHybridStateMachine("test_sm", DUMMY_DELTA_TIME, sm_clock);
	factory->addLocations(*hy, {"ON", "OFF"});
	std::vector<std::string> loc_label = hy->getVertexLabels();
	ASSERT_EQ(loc_label[0], "PSEUDO_START");
	ASSERT_EQ(loc_label[1], "PSEUDO_END");
	ASSERT_EQ(loc_label[2], "ON");
	ASSERT_EQ(loc_label[3], "OFF");
	ASSERT_EQ(hy->getNumberOfLocations(), 4);
	// Set transitions
	addTransitionsTestSM_OnOff(factory, hy);
	// Check step
	ASSERT_EQ(hy->getCurrentLocation()->getLabel(), "PSEUDO_START");
	// All transitions are enabled
	DiscreteEventPipeline<unsigned long, DummyZeroClock> pipeline_test;
	pipeline_test.setEventMapping(factory->getEventMapping());
	pipeline_test.addStateMachine(hy);
	pipeline_test.propagateEvent("StartEvent", 0);
	ASSERT_EQ(hy->step(), HybridStateStepResult::TRANSITED);
	ASSERT_EQ(hy->getCurrentLocation()->getLabel(), "ON");
	// Should not crash
	ASSERT_EQ(hy->step(), HybridStateStepResult::EMPTY_QUEUE);
	ASSERT_EQ(hy->getCurrentLocation()->getLabel(), "ON");
}

/*
 * @test: Testing the absence of a clock (test exception)
 * @req: REQ.HYSM.OP.2: the SM shall not step on absent clock and throw an exception
 */
TEST(TestHybridStateMachine, TestAbsentClock)
{
	using namespace rei::node;
	HybridStateMachineFactory<unsigned long, DummyZeroClock>* factory =
			HybridStateMachineFactory<unsigned long, DummyZeroClock>::getInstance();
	std::shared_ptr<DummyZeroClock> sm_clock = std::make_shared<DummyZeroClock>();
	std::shared_ptr<HybridStateMachine<unsigned long, DummyZeroClock>> hy =
		factory->createHybridStateMachine("test_sm", DUMMY_DELTA_TIME, sm_clock);
	factory->addLocations(*hy, {"ON", "OFF"});
	std::vector<std::string> loc_label = hy->getVertexLabels();
	ASSERT_EQ(loc_label[0], "PSEUDO_START");
	ASSERT_EQ(loc_label[1], "PSEUDO_END");
	ASSERT_EQ(loc_label[2], "ON");
	ASSERT_EQ(loc_label[3], "OFF");
	ASSERT_EQ(hy->getNumberOfLocations(), 4);
	// Add transitions
	addTransitionsTestSM_OnOff<DummyZeroClock>(factory, hy);
	// Check step
	ASSERT_EQ(hy->getCurrentLocation()->getLabel(), "PSEUDO_START");
	// All transitions are enabled
	DiscreteEventPipeline<unsigned long, DummyZeroClock> pipeline_test;
	pipeline_test.setEventMapping(factory->getEventMapping());
	pipeline_test.addStateMachine(hy);
	pipeline_test.propagateEvent("StartEvent", 0, 0);
	ASSERT_EQ(hy->step(), HybridStateStepResult::TRANSITED);
	ASSERT_EQ(hy->getCurrentLocation()->getLabel(), "ON");
	// Should not crash
	hy->setClock(nullptr);
	ASSERT_THROW(hy->step(), std::runtime_error);
}

/*
 * @test: Test timeout of events that are coming late or to soon.
 */
TEST(TestHybridStateMachine, TestTimeoutEvent)
{
	using namespace rei::node;
	HybridStateMachineFactory<unsigned long, DummyConstantClock>* factory =
			HybridStateMachineFactory<unsigned long, DummyConstantClock>::getInstance();
	std::shared_ptr<DummyConstantClock> sm_clock = std::make_shared<DummyConstantClock>();
	std::shared_ptr<HybridStateMachine<unsigned long, DummyConstantClock>> hy =
		factory->createHybridStateMachine("test_sm", DUMMY_DELTA_TIME, sm_clock);
	factory->addLocations(*hy, {"ON", "OFF"});
	ASSERT_EQ(hy->getNumberOfLocations(), 4);
	// Add transitions
	addTransitionsTestSM_OnOff<DummyConstantClock>(factory, hy);
	// Check step
	ASSERT_EQ(hy->getCurrentLocation()->getLabel(), "PSEUDO_START");
	DiscreteEventPipeline<unsigned long, DummyConstantClock> pipeline_test;
	pipeline_test.setEventMapping(factory->getEventMapping());
	pipeline_test.addStateMachine(hy);
	pipeline_test.propagateEvent("StartEvent", DUMMY_DELTA_TIME);
	ASSERT_EQ(hy->step(), HybridStateStepResult::PROCESS_TIMEOUT);
	ASSERT_EQ(hy->getCurrentLocation()->getLabel(), "PSEUDO_START");
	pipeline_test.propagateEvent("StartEvent", 2*DUMMY_DELTA_TIME);
	ASSERT_EQ(hy->step(), HybridStateStepResult::PROCESS_TIMEOUT);
	ASSERT_EQ(hy->getCurrentLocation()->getLabel(), "PSEUDO_START");
	pipeline_test.propagateEvent("StartEvent", 6*DUMMY_DELTA_TIME);
	ASSERT_EQ(hy->step(), HybridStateStepResult::SHIFTED_TIMESTAMP);
	ASSERT_EQ(hy->getCurrentLocation()->getLabel(), "PSEUDO_START");
	pipeline_test.propagateEvent("StartEvent", 3.1*DUMMY_DELTA_TIME);
	ASSERT_EQ(hy->step(), HybridStateStepResult::TRANSITED);
	ASSERT_EQ(hy->getCurrentLocation()->getLabel(), "ON");
}

/*
 * @test: Check one false transit
 */
TEST(TestHybridStateMachine, TestAssignTransitionFalseGuard)
{
	using namespace rei::node;
	HybridStateMachineFactory<unsigned long, DummyZeroClock>* factory =
			HybridStateMachineFactory<unsigned long, DummyZeroClock>::getInstance();
	std::shared_ptr<DummyZeroClock> sm_clock = std::make_shared<DummyZeroClock>();
	std::shared_ptr<HybridStateMachine<unsigned long, DummyZeroClock>> hy =
		factory->createHybridStateMachine("test_sm", DUMMY_DELTA_TIME, sm_clock);
	factory->addLocations(*hy, {"ON", "OFF"});
	// Set transitions
	addTransitionsTestSM_OnOffTransitionList(factory, hy);
	std::vector<std::string> loc_label = hy->getVertexLabels();
	ASSERT_EQ(loc_label[0], "PSEUDO_START");
	ASSERT_EQ(loc_label[1], "PSEUDO_END");
	ASSERT_EQ(loc_label[2], "ON");
	ASSERT_EQ(loc_label[3], "OFF");
	ASSERT_EQ(hy->getNumberOfLocations(), 4);
	factory->assignTransitionGuardFunction(*hy, "OFF", "ON", []()->bool { return false; });
	// Check step
	ASSERT_EQ(hy->getCurrentLocation()->getLabel(), "PSEUDO_START");
	// All transitions are enabled
	DiscreteEventPipeline<unsigned long, DummyZeroClock> pipeline_test;
	pipeline_test.setEventMapping(factory->getEventMapping());
	pipeline_test.addStateMachine(hy);
	pipeline_test.propagateEvent("StartEvent", 0);
	ASSERT_EQ(hy->step(), HybridStateStepResult::TRANSITED);
	ASSERT_EQ(hy->getCurrentLocation()->getLabel(), "ON");
	// Transition to another state
	pipeline_test.propagateEvent("StartEvent", 0);
	ASSERT_EQ(hy->step(), HybridStateStepResult::NO_TRANSITION);
	ASSERT_EQ(hy->getCurrentLocation()->getLabel(), "ON");
	pipeline_test.propagateEvent("foo", 0);
	ASSERT_EQ(hy->step(), HybridStateStepResult::TRANSITED);
	ASSERT_EQ(hy->getCurrentLocation()->getLabel(), "OFF");
	pipeline_test.propagateEvent("bar", 0);
	ASSERT_EQ(hy->step(), HybridStateStepResult::NO_TRANSITION);
	ASSERT_EQ(hy->getCurrentLocation()->getLabel(), "OFF");
}

int main(int argc, char** argv)
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
