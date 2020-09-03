/*
 * test_basic_hybridstatemachine.cpp
 *
 *  Created on: Sep 3, 2020
 *      Author: kyberszittya
 */


#include <gtest/gtest.h>

#include "rei_construct_elements/rei_hybrid_state_machine.hpp"

struct DummySignal
{
	unsigned int sig_id;
};

TEST(HybridStateMachineFactory, TestBasicConstruction)
{
	using namespace rei::node;
	HybridStateMachineFactory<DummySignal>* factory = HybridStateMachineFactory<DummySignal>::getInstance();
	HybridStateMachine<DummySignal> hy = factory->createHybridStateMachine();
	std::vector<std::string> loc_label = hy.getLocationLabels();
	ASSERT_EQ(loc_label[0], "PSEUDO_START");
	ASSERT_EQ(loc_label[1], "PSEUDO_END");
	ASSERT_EQ(hy.getNumberOfLocations(), 2);
	ASSERT_EQ(hy.getCurrentLocation()->getLabel(), "PSEUDO_START");
}

TEST(HybridStateMachineFactory, TestAssignStatesBasicConstruction)
{
	using namespace rei::node;
	HybridStateMachineFactory<DummySignal>* factory = HybridStateMachineFactory<DummySignal>::getInstance();
	HybridStateMachine<DummySignal> hy = factory->createHybridStateMachine();
	factory->addLocations(hy, {"WAITING", "ENABLED", "DISABLED"});
	std::vector<std::string> loc_label = hy.getLocationLabels();
	ASSERT_EQ(loc_label[0], "PSEUDO_START");
	ASSERT_EQ(loc_label[1], "PSEUDO_END");
	ASSERT_EQ(loc_label[2], "WAITING");
	ASSERT_EQ(loc_label[3], "ENABLED");
	ASSERT_EQ(loc_label[4], "DISABLED");
	ASSERT_EQ(hy.getCurrentLocation()->getLabel(), "PSEUDO_START");
	ASSERT_EQ(hy.getNumberOfLocations(), 5);
}

TEST(HybridStateMachineFactory, TestAssignTransitionBasicConstruction)
{
	using namespace rei::node;
	HybridStateMachineFactory<DummySignal>* factory = HybridStateMachineFactory<DummySignal>::getInstance();
	HybridStateMachine<DummySignal> hy = factory->createHybridStateMachine();
	factory->addLocations(hy, {"ON", "OFF"});
	std::vector<std::string> loc_label = hy.getLocationLabels();
	ASSERT_EQ(loc_label[0], "PSEUDO_START");
	ASSERT_EQ(loc_label[1], "PSEUDO_END");
	ASSERT_EQ(loc_label[2], "ON");
	ASSERT_EQ(loc_label[3], "OFF");
	ASSERT_EQ(hy.getNumberOfLocations(), 4);
	// Set transitions
	factory->addDiscreteTransition(hy, "StartEvent", std::pair<std::string, std::string>("PSEUDO_START","ON"));
	factory->addDiscreteTransition(hy, "foo", std::pair<std::string, std::string>("ON", "OFF"));
	factory->addDiscreteTransition(hy, "bar", std::pair<std::string, std::string>("OFF", "ON"));
	factory->addDiscreteTransition(hy, "baz", std::pair<std::string, std::string>("OFF", "PSEUDO_END"));
	// Implement step
	ASSERT_EQ(hy.getCurrentLocation()->getLabel(), "PSEUDO_START");

}


int main(int argc, char** argv)
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
