/*
 * test_common.hpp
 *
 *  Created on: Feb 10, 2020
 *      Author: kyberszittya
 */

#ifndef TEST_TEST_COMMON_HPP_
#define TEST_TEST_COMMON_HPP_

#include <gtest/gtest.h>

#include <hotaru_node_elements/state_machine/trajectory_signal.hpp>
#include <hotaru_node_elements/state_machine/trajectory_statemachine.hpp>

#include <memory>
#include <iostream>


namespace hotaru
{


namespace test_local_planner_state_machine
{

class DummyLocalPlannerStateMachineGuard: public Interface_GuardLocalPlanner
{
public:
	virtual bool guard_Relay2ReplanningState() override
	{
		return true;
	}

	virtual bool guard_Replanning2RelayState() override
	{
		return true;
	}

	virtual bool guard_Relay2Waiting() override
	{
		return true;
	}

	virtual bool guard_Waiting2Relay() override
	{
		return true;
	}
};

}

}
#endif /* TEST_TEST_COMMON_HPP_ */
