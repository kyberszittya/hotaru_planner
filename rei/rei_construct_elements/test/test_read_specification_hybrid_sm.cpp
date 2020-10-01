/*
 * test_read_specification_hybrid_sm.hpp
 *
 *  Created on: Sep 10, 2020
 *      Author: kyberszittya
 */

#include <gtest/gtest.h>

#include <rei_construct_elements/specread/read_hybrid_system_spec.hpp>

#include <ros/package.h>

#include "test_common.hpp"

/**
 * @test Read basic hybrid state machine
 */
TEST(TestReadSpecificationXML, TestSimpleStateMachine)
{
	std::string path = ros::package::getPath("rei_construct_elements");
	using namespace rei;
	specification::ReadSpecXmlHybridStateMachine<unsigned long, DummyZeroClock> readspecxml;
	std::shared_ptr<DummyZeroClock> sm_clock = std::make_shared<DummyZeroClock>();
	readspecxml.readXmlDescription(path+"/test/test-data/parking.xml");
	std::shared_ptr<node::HybridStateMachine<unsigned long, DummyZeroClock>> hy = readspecxml.getCurrentHybridStateMachine();
	std::vector<std::string> loc_label = hy->getVertexLabels();
	ASSERT_EQ(loc_label[0], "PSEUDO_START");
	ASSERT_EQ(loc_label[1], "PSEUDO_END");
	ASSERT_EQ(loc_label[2], "Idle");
	ASSERT_EQ(loc_label[3], "Searching");
	ASSERT_EQ(loc_label[4], "Parking");
	ASSERT_EQ(loc_label[5], "OptimizeParking");
	hy->setClock(sm_clock);
	// Test notification events
	node::DiscreteEventPipeline<unsigned long, DummyZeroClock> pipeline_test;
	pipeline_test.setEventMapping(readspecxml.getEventMapping());
	pipeline_test.addStateMachine(hy);
	pipeline_test.propagateEvent("SwitchBehaviorSignal", 0);
	ASSERT_EQ(hy->step(), node::HybridStateStepResult::TRANSITED);
	ASSERT_EQ(hy->getCurrentLocation()->getLabel(), "Idle");
	// Transition to another state
	pipeline_test.propagateEvent("ParkingStartSignal", 0);
	ASSERT_EQ(hy->step(), node::HybridStateStepResult::TRANSITED);
	ASSERT_EQ(hy->getCurrentLocation()->getLabel(), "Searching");
	pipeline_test.propagateEvent("ParkingPointDetectedSignal", 0);
	ASSERT_EQ(hy->step(), node::HybridStateStepResult::TRANSITED);
	ASSERT_EQ(hy->getCurrentLocation()->getLabel(), "Parking");
	pipeline_test.propagateEvent("ParkingPointCloseProximitySignal", 0);
	ASSERT_EQ(hy->step(), node::HybridStateStepResult::TRANSITED);
	ASSERT_EQ(hy->getCurrentLocation()->getLabel(), "OptimizeParking");
	pipeline_test.propagateEvent("OptimalParkingSignal", 0);
	ASSERT_EQ(hy->step(), node::HybridStateStepResult::TRANSITED);
	ASSERT_EQ(hy->getCurrentLocation()->getLabel(), "Idle");
	pipeline_test.propagateEvent("ExitBehaviorSignal", 0);
	ASSERT_EQ(hy->step(), node::HybridStateStepResult::TRANSITED);
	ASSERT_EQ(hy->getCurrentLocation()->getLabel(), "PSEUDO_END");
}

int main(int argc, char** argv)
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
