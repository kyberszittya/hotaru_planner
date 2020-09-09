/*
 * example_hybrid_state_machine.cpp
 *
 *  Created on: Sep 9, 2020
 *      Author: kyberszittya
 */

#include <rei_construct_elements/ros1/rei_hybrid_control_node_ros1.hpp>
#include <rei_construct_elements/hybrid_dynamic_system/rei_hybrid_state_machine_factory.hpp>
#include <rei_construct_elements/ros1/rei_hybrid_pipeline_ros1.hpp>
#include <rei_construct_elements/rei_control_system.hpp>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ros_hysm_example");
	using namespace rei::node;
	HybridStateMachineFactoryRos1* factory = HybridStateMachineFactoryRos1::getInstance();
	std::shared_ptr<RosClock> clock = std::make_shared<RosClock>();
	HybridStateMachinePtrRos1 hysm = factory->createHybridStateMachine("example_hysm", 1000000000, clock);
	factory->addLocations(*hysm, {"Idle", "Searching", "Parking", "OptimizeParking"});
	factory->setTerminalLocations(hysm, "SwitchBehaviorSignal", "Idle", "ExitBehaviorSignal", "Idle");
	factory->addDiscreteTransitions(*hysm,
			{{"ParkingStartSignal", "Idle", "Searching"},
			{"InfeasibleParkingPoint", "Searching", "Idle"},
			{"ParkingPointDetectedSignal", "Searching", "Parking"},
			{"ParkingPointCloseProximitySignal", "Parking", "OptimizeParking"},
			{"OptimalParkingSignal", "OptimizeParking", "Idle"},
			{"UnoptimalParkingSignal", "OptimizeParking", "Idle"}});
	ROS_INFO("Created an example hybrid automata");
	NotificationContextPtrRos1 notification_context = std::make_shared<NotificationContextRos1>();
	ros1::DiscreteEventPipelinePtrRos1 event_pipeline = std::make_shared<ros1::DiscreteEventPipelineRos1>();
	event_pipeline->setEventMapping(factory->getEventMapping());
	std::shared_ptr<ros::NodeHandle> nh = std::make_shared<ros::NodeHandle>();
	HybridControlNodeRos1 hybrid_control_node(nh);

	hybrid_control_node.setHybridSystem(hysm);
	hybrid_control_node.setClock(clock);
	hybrid_control_node.setDiscreteEventPipeline(event_pipeline);
	hybrid_control_node.setNotificationContext(notification_context);
	ROS_INFO("Initializing signaling interface");
	hybrid_control_node.init_control_signal_interface();
	hybrid_control_node.startTimer(0.1);
	ROS_INFO("Set up ROS interface of hybrid system");
	ros::spin();
	return 0;
}
