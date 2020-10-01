/*
 * test_common.hpp
 *
 *  Created on: Sep 8, 2020
 *      Author: kyberszittya
 */

#ifndef TEST_TEST_COMMON_HPP_
#define TEST_TEST_COMMON_HPP_

#include <rei_construct_elements/hybrid_dynamic_system/rei_hybrid_state_machine.hpp>
#include <rei_construct_elements/hybrid_dynamic_system/rei_hybrid_state_machine_factory.hpp>


constexpr unsigned long DUMMY_DELTA_TIME = 10000000;

constexpr unsigned long CLOCK_CONSTANT = DUMMY_DELTA_TIME*4;


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

class DummyConstantClock: public rei::util::ClockInterface<unsigned long>
{
public:
	virtual unsigned long getCurrentTime() override
	{
		return CLOCK_CONSTANT;
	}
};



/**
 * Functions related to the following hybrid automata (TestSM_OnOff)
 *
 *  0 ->  ON
 *       ^  |
 *       |  ˇ
 *  1 <-  OFF
 */

template<class Clock> void addTransitionsTestSM_OnOff(
		rei::node::HybridStateMachineFactory<unsigned long, Clock>* factory,
		std::shared_ptr<rei::node::HybridStateMachine<unsigned long, Clock>> hy)
{
	// Set transitions
	factory->setTerminalLocations(hy, "StartEvent", "ON", "baz", "OFF");
	factory->addDiscreteTransition(*hy, "foo", std::pair<std::string, std::string>("ON", "OFF"));
	factory->addDiscreteTransition(*hy, "bar", std::pair<std::string, std::string>("OFF", "ON"));
}

template<class Clock> void addTransitionsTestSM_OnOffTransitionList(
		rei::node::HybridStateMachineFactory<unsigned long, Clock>* factory,
		std::shared_ptr<rei::node::HybridStateMachine<unsigned long, Clock>> hy)
{
	// Set transitions
	factory->setTerminalLocations(hy, "StartEvent", "ON", "baz", "OFF");
	factory->addDiscreteTransitions(*hy,
			{{"foo", "ON", "OFF"},
			{"bar", "OFF", "ON"}});
	factory->assignTransitionGuardFunction(*hy, "ON", "OFF",  []()->bool{ return true; });
	factory->assignTransitionGuardFunction(*hy, "OFF", "ON",  []()->bool{ return true; });
}

#endif /* TEST_TEST_COMMON_HPP_ */
