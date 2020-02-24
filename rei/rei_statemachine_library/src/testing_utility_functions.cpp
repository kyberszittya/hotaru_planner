/*
 * testing_utility_functions.cpp
 *
 *  Created on: Feb 24, 2020
 *      Author: kyberszittya
 */

#include <rei_statemachine_library/testing/test_common.hpp>

namespace rei
{


namespace test_sync_state_machine
{

void dummy_start_function()
{
	std::cout << "STARTED SM\n";
}

void dummy_allmessages_received_function()
{
	std::cout << "All messages received\n";
}

}

}
