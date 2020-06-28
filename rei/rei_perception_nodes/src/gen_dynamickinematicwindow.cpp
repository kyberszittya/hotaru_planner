#include <rei_perception_nodes/gen_dynamickinematicwindow.hpp>

namespace rei {

InterfaceRos_DynamicKinematicWindow::~InterfaceRos_DynamicKinematicWindow() {}

bool InterfaceRos_DynamicKinematicWindow::initTimeoutStateMachine()
{
	// Sync state machine initialization
	return true;
}

bool InterfaceRos_DynamicKinematicWindow::initMiddleware(const bool debug, const bool bypass_behavior)
{
	/// Initialize internal pubsub state
	pubsubstate = std::unique_ptr<StateDynamicKinematicWindow>(new StateDynamicKinematicWindow(debug, bypass_behavior));
	if (pubsubstate==nullptr)
	{
		return false;
	}
	/// Initialize ROS publishers
	port_grid_map = nh->advertise<grid_map_msgs::GridMap>("dynamic_window", 10);
	/// Initialize ROS subscribers
	port_vehicle_status = nh->subscribe("vehicle_status", 10, &InterfaceRos_DynamicKinematicWindow::cbPort_vehicle_status, this);
	port_motion_dyn_param = nh->subscribe("motion_dynamic_constraints", 10, &InterfaceRos_DynamicKinematicWindow::cbPort_motion_dyn_param, this);
	return true;
}



void InterfaceRos_DynamicKinematicWindow::cbPort_vehicle_status(const autoware_msgs::VehicleStatus::ConstPtr& msg)
{
	pubsubstate->msg_port_vehicle_status = *msg;
	executeUpdatevehiclestatus(msg);
	
}
void InterfaceRos_DynamicKinematicWindow::cbPort_motion_dyn_param(const rei_monitoring_msgs::MotionDynamicConstraints::ConstPtr& msg)
{
	pubsubstate->msg_port_motion_dyn_param = *msg;
	executeUpdateconstraints(msg);
	
}

void InterfaceRos_DynamicKinematicWindow::publishDynamic_window()
{
	port_grid_map.publish(pubsubstate->msg_port_grid_map);
}

}
