/*
 * hybrid_state_machine_reader.cpp
 *
 *  Created on: Sep 10, 2020
 *      Author: kyberszittya
 */


#include <rei_construct_elements/specread/read_hybrid_system_spec.hpp>
#include <rei_construct_elements/hybrid_dynamic_system/rei_hybrid_state_machine.hpp>
#include <rei_construct_elements/ros1/rei_hybrid_control_node_ros1.hpp>
#include <rei_construct_elements/ros1/rei_hybrid_pipeline_ros1.hpp>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ros_hysm_example_reader");
	using namespace rei;
	specification::ReadSpecXmlHybridStateMachine<unsigned long, node::RosClock> readspecxml;
	std::shared_ptr<node::RosClock> sm_clock = std::make_shared<node::RosClock>();
	std::shared_ptr<ros::NodeHandle> nh = std::make_shared<ros::NodeHandle>();
	std::shared_ptr<ros::NodeHandle> private_nh = std::make_shared<ros::NodeHandle>("~");
	std::string param_filename;
	if (!private_nh->getParam("filename", param_filename))
	{
		ROS_FATAL("File parameter not loaded");
		ROS_FATAL_STREAM("File not found: " << param_filename);
		return -6;
	}
	ROS_INFO("Loaded file");
	readspecxml.readXmlDescription(param_filename);
	std::shared_ptr<node::HybridStateMachine<unsigned long, node::RosClock>> hy =
			readspecxml.getCurrentHybridStateMachine();
	node::ros1::DiscreteEventPipelinePtrRos1 event_pipeline =
			std::make_shared<node::ros1::DiscreteEventPipelineRos1>();
	node::NotificationContextPtrRos1 notification_context =
			std::make_shared<node::NotificationContextRos1>();
	event_pipeline->setEventMapping(readspecxml.getEventMapping());
	node::HybridControlNodeRos1 hybrid_control_node(nh);
	hybrid_control_node.setHybridSystem(hy);
	hybrid_control_node.setClock(sm_clock);
	hybrid_control_node.setDiscreteEventPipeline(event_pipeline);
	hybrid_control_node.setNotificationContext(notification_context);
	ROS_INFO("Initialized ROS1 HYSM signaling interface");
	hybrid_control_node.init_control_signal_interface();
	hybrid_control_node.startTimer(0.1);
	ROS_INFO("ROS1 - interface started");
	ros::spin();
	return 0;
}

