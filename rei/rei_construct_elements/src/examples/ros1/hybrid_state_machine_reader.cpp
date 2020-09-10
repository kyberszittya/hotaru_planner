/*
 * hybrid_state_machine_reader.cpp
 *
 *  Created on: Sep 10, 2020
 *      Author: kyberszittya
 */


#include <rei_construct_elements/specread/read_hybrid_system_spec.hpp>
#include <rei_construct_elements/hybrid_dynamic_system/rei_hybrid_state_machine.hpp>
#include <rei_construct_elements/ros1/rei_hybrid_control_node_ros1.hpp>

int main(int argc, char** argv)
{
	using namespace rei;
	specification::ReadSpecXmlHybridStateMachine<unsigned long, node::RosClock> readspecxml;
	std::shared_ptr<node::RosClock> sm_clock = std::make_shared<node::RosClock>();
	readspecxml.readXmlDescription("example/parking.xml");
	std::shared_ptr<node::HybridStateMachine<unsigned long, node::RosClock>> hy = readspecxml.getCurrentHybridStateMachine();
	std::vector<std::string> loc_label = hy->getVertexLabels();


	return 0;
}

