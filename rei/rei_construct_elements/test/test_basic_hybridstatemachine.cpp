/*
 * test_basic_hybridstatemachine.cpp
 *
 *  Created on: Sep 3, 2020
 *      Author: kyberszittya
 */


#include <gtest/gtest.h>

#include <rei_construct_elements/hybrid_dynamic_system/rei_hybrid_state_machine.hpp>

struct DummySignal
{
	unsigned int sig_id;
};

class DummyZeroClock: public rei::util::ClockInterface<unsigned long>
{
public:
	virtual unsigned long getCurrentTime() override
	{
		return 0;
	}
};

constexpr unsigned long DUMMY_DELTA_TIME = 10000000;

TEST(HybridStateMachineFactory, TestBasicConstruction)
{
	using namespace rei::node;
	HybridStateMachineFactory<unsigned long, DummyZeroClock>* factory = HybridStateMachineFactory<unsigned long, DummyZeroClock>::getInstance();
	std::shared_ptr<DummyZeroClock> sm_clock = std::make_shared<DummyZeroClock>();
	std::shared_ptr<HybridStateMachine<unsigned long, DummyZeroClock>> hy =
		factory->createHybridStateMachine("test_sm", DUMMY_DELTA_TIME, sm_clock);
	std::vector<std::string> loc_label = hy->getLocationLabels();
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

TEST(HybridStateMachineFactory, TestAssignStatesBasicConstruction)
{
	using namespace rei::node;
	HybridStateMachineFactory<unsigned long, DummyZeroClock>* factory = HybridStateMachineFactory<unsigned long, DummyZeroClock>::getInstance();
	std::shared_ptr<DummyZeroClock> sm_clock = std::make_shared<DummyZeroClock>();
	std::shared_ptr<HybridStateMachine<unsigned long, DummyZeroClock>> hy =
		factory->createHybridStateMachine("test_sm", DUMMY_DELTA_TIME, sm_clock);
	factory->addLocations(*hy, {"WAITING", "ENABLED", "DISABLED"});
	std::vector<std::string> loc_label = hy->getLocationLabels();
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

TEST(HybridStateMachineFactory, TestAssignTransitionBasicConstruction)
{
	using namespace rei::node;
	HybridStateMachineFactory<unsigned long, DummyZeroClock>* factory =
			HybridStateMachineFactory<unsigned long, DummyZeroClock>::getInstance();
	std::shared_ptr<DummyZeroClock> sm_clock = std::make_shared<DummyZeroClock>();
	std::shared_ptr<HybridStateMachine<unsigned long, DummyZeroClock>> hy =
		factory->createHybridStateMachine("test_sm", DUMMY_DELTA_TIME, sm_clock);
	factory->addLocations(*hy, {"ON", "OFF"});
	std::vector<std::string> loc_label = hy->getLocationLabels();
	ASSERT_EQ(loc_label[0], "PSEUDO_START");
	ASSERT_EQ(loc_label[1], "PSEUDO_END");
	ASSERT_EQ(loc_label[2], "ON");
	ASSERT_EQ(loc_label[3], "OFF");
	ASSERT_EQ(hy->getNumberOfLocations(), 4);
	// Set transitions
	factory->addDiscreteTransition(*hy, "StartEvent", std::pair<std::string, std::string>("PSEUDO_START","ON"));
	factory->addDiscreteTransition(*hy, "foo", std::pair<std::string, std::string>("ON", "OFF"));
	factory->addDiscreteTransition(*hy, "bar", std::pair<std::string, std::string>("OFF", "ON"));
	factory->addDiscreteTransition(*hy, "baz", std::pair<std::string, std::string>("OFF", "PSEUDO_END"));

	// Check step
	ASSERT_EQ(hy->getCurrentLocation()->getLabel(), "PSEUDO_START");
	// All transitions are enabled
	DiscreteEventPipeline<unsigned long, DummyZeroClock> pipeline_test;
	pipeline_test.addStateMachine(hy);
	pipeline_test.propagateEvent("StartEvent", 0, 0);
	ASSERT_EQ(hy->step(), HybridStateStepResult::TRANSITED);
	ASSERT_EQ(hy->getCurrentLocation()->getLabel(), "ON");
	// Transition to another state
	pipeline_test.propagateEvent("StartEvent", 0, 0);
	ASSERT_EQ(hy->step(), HybridStateStepResult::NO_TRANSITION);
	ASSERT_EQ(hy->getCurrentLocation()->getLabel(), "ON");
	pipeline_test.propagateEvent("foo", 1, 0);
	ASSERT_EQ(hy->step(), HybridStateStepResult::TRANSITED);
	ASSERT_EQ(hy->getCurrentLocation()->getLabel(), "OFF");
	pipeline_test.propagateEvent("bar", 2, 0);
	ASSERT_EQ(hy->step(), HybridStateStepResult::TRANSITED);
	ASSERT_EQ(hy->getCurrentLocation()->getLabel(), "ON");
	pipeline_test.propagateEvent("foo", 1, 0);
	ASSERT_EQ(hy->step(), HybridStateStepResult::TRANSITED);
	ASSERT_EQ(hy->getCurrentLocation()->getLabel(), "OFF");
	pipeline_test.propagateEvent("baz", 3, 0);
	ASSERT_EQ(hy->step(), HybridStateStepResult::TRANSITED);
	ASSERT_EQ(hy->getCurrentLocation()->getLabel(), "PSEUDO_END");
}

/*
 * @test: Test on multiple steps (ensure it's working even on empty event stack)
 * @req: REQ.HYSM.OP.1: the SM shall not transit on empty event stack
 */
TEST(HybridStateMachineFactory, TestMultipleSteps)
{
	using namespace rei::node;
	HybridStateMachineFactory<unsigned long, DummyZeroClock>* factory =
			HybridStateMachineFactory<unsigned long, DummyZeroClock>::getInstance();
	std::shared_ptr<DummyZeroClock> sm_clock = std::make_shared<DummyZeroClock>();
	std::shared_ptr<HybridStateMachine<unsigned long, DummyZeroClock>> hy =
		factory->createHybridStateMachine("test_sm", DUMMY_DELTA_TIME, sm_clock);
	factory->addLocations(*hy, {"ON", "OFF"});
	std::vector<std::string> loc_label = hy->getLocationLabels();
	ASSERT_EQ(loc_label[0], "PSEUDO_START");
	ASSERT_EQ(loc_label[1], "PSEUDO_END");
	ASSERT_EQ(loc_label[2], "ON");
	ASSERT_EQ(loc_label[3], "OFF");
	ASSERT_EQ(hy->getNumberOfLocations(), 4);
	// Set transitions
	factory->addDiscreteTransition(*hy, "StartEvent", std::pair<std::string, std::string>("PSEUDO_START","ON"));
	factory->addDiscreteTransition(*hy, "foo", std::pair<std::string, std::string>("ON", "OFF"));
	factory->addDiscreteTransition(*hy, "bar", std::pair<std::string, std::string>("OFF", "ON"));
	factory->addDiscreteTransition(*hy, "baz", std::pair<std::string, std::string>("OFF", "PSEUDO_END"));
	// Check step
	ASSERT_EQ(hy->getCurrentLocation()->getLabel(), "PSEUDO_START");
	// All transitions are enabled
	DiscreteEventPipeline<unsigned long, DummyZeroClock> pipeline_test;
	pipeline_test.addStateMachine(hy);
	pipeline_test.propagateEvent("StartEvent", 0, 0);
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
TEST(HybridStateMachineFactory, TestAbsentClock)
{
	using namespace rei::node;
	HybridStateMachineFactory<unsigned long, DummyZeroClock>* factory =
			HybridStateMachineFactory<unsigned long, DummyZeroClock>::getInstance();
	std::shared_ptr<DummyZeroClock> sm_clock = std::make_shared<DummyZeroClock>();
	std::shared_ptr<HybridStateMachine<unsigned long, DummyZeroClock>> hy =
		factory->createHybridStateMachine("test_sm", DUMMY_DELTA_TIME, sm_clock);
	factory->addLocations(*hy, {"ON", "OFF"});
	std::vector<std::string> loc_label = hy->getLocationLabels();
	ASSERT_EQ(loc_label[0], "PSEUDO_START");
	ASSERT_EQ(loc_label[1], "PSEUDO_END");
	ASSERT_EQ(loc_label[2], "ON");
	ASSERT_EQ(loc_label[3], "OFF");
	ASSERT_EQ(hy->getNumberOfLocations(), 4);
	// Set transitions
	factory->addDiscreteTransition(*hy, "StartEvent", std::pair<std::string, std::string>("PSEUDO_START","ON"));
	factory->addDiscreteTransition(*hy, "foo", std::pair<std::string, std::string>("ON", "OFF"));
	factory->addDiscreteTransition(*hy, "bar", std::pair<std::string, std::string>("OFF", "ON"));
	factory->addDiscreteTransition(*hy, "baz", std::pair<std::string, std::string>("OFF", "PSEUDO_END"));
	// Check step
	ASSERT_EQ(hy->getCurrentLocation()->getLabel(), "PSEUDO_START");
	// All transitions are enabled
	DiscreteEventPipeline<unsigned long, DummyZeroClock> pipeline_test;
	pipeline_test.addStateMachine(hy);
	pipeline_test.propagateEvent("StartEvent", 0, 0);
	ASSERT_EQ(hy->step(), HybridStateStepResult::TRANSITED);
	ASSERT_EQ(hy->getCurrentLocation()->getLabel(), "ON");
	// Should not crash
	hy->setClock(nullptr);
	ASSERT_THROW(hy->step(), std::runtime_error);
}


int main(int argc, char** argv)
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
