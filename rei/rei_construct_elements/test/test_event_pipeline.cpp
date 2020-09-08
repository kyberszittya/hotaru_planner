/*
 * test_event_pipeline.cpp
 *
 *  Created on: Sep 8, 2020
 *      Author: kyberszittya
 */


#include <gtest/gtest.h>

#include "test_common.hpp"

TEST(TestEventPipeline, TestReceivingPipeline)
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
	addTransitionsTestSM_OnOff(factory, hy);
	// Setup event pipeline
	DiscreteEventPipeline<unsigned long, DummyZeroClock> pipeline_test;
	pipeline_test.addStateMachine(hy);
	std::shared_ptr<NotificationContext<unsigned long>> notification_context =
		std::make_shared<NotificationContext<unsigned long>>();
	hy->setNotificationContext(notification_context);
	pipeline_test.propagateEvent("StartEvent", 0, 0);
	hy->step();
	pipeline_test.propagateEvent("foo", 1, 0);
	hy->step();
	pipeline_test.propagateEvent("bar", 2, 0);
	hy->step();
	pipeline_test.propagateEvent("foo", 1, 0);
	hy->step();
	pipeline_test.propagateEvent("baz", 3, 0);
	hy->step();
	// Location events
	ASSERT_EQ(notification_context->popLocationEvent()->getEventName(), "PSEUDO_END");
	ASSERT_EQ(notification_context->popLocationEvent()->getEventName(), "OFF");
	ASSERT_EQ(notification_context->popLocationEvent()->getEventName(), "ON");
	ASSERT_EQ(notification_context->popLocationEvent()->getEventName(), "OFF");
	ASSERT_EQ(notification_context->popLocationEvent()->getEventName(), "ON");
	// Transition events
	ASSERT_EQ(notification_context->popTransitionEvent()->getEventName(), "OFF->PSEUDO_END");
	ASSERT_EQ(notification_context->popTransitionEvent()->getEventName(), "ON->OFF");
	ASSERT_EQ(notification_context->popTransitionEvent()->getEventName(), "OFF->ON");
	ASSERT_EQ(notification_context->popTransitionEvent()->getEventName(), "ON->OFF");
	ASSERT_EQ(notification_context->popTransitionEvent()->getEventName(), "PSEUDO_START->ON");
}

TEST(TestEventPipeline, TestReceivingPipelineNoEvents)
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
	addTransitionsTestSM_OnOff(factory, hy);
	// Setup event pipeline
	DiscreteEventPipeline<unsigned long, DummyZeroClock> pipeline_test;
	pipeline_test.addStateMachine(hy);
	std::shared_ptr<NotificationContext<unsigned long>> notification_context =
		std::make_shared<NotificationContext<unsigned long>>();
	hy->setNotificationContext(notification_context);
	ASSERT_EQ(notification_context->popLocationEvent(), nullptr);
	ASSERT_EQ(notification_context->popLocationEvent(), nullptr);
	ASSERT_EQ(notification_context->popTransitionEvent(), nullptr);
}

int main(int argc, char** argv)
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
