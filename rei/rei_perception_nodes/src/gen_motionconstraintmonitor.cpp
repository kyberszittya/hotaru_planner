#include <rei_perception_nodes/gen_motionconstraintmonitor.hpp>

namespace rei {

InterfaceRos_MotionConstraintMonitor::~InterfaceRos_MotionConstraintMonitor() {}

bool InterfaceRos_MotionConstraintMonitor::initTimeoutStateMachine()
{
	// Sync state machine initialization
	return true;
}

bool InterfaceRos_MotionConstraintMonitor::initMiddleware(const bool debug, const bool bypass_behavior)
{
	/// Initialize internal pubsub state
	pubsubstate = std::unique_ptr<StateMotionConstraintMonitor>(new StateMotionConstraintMonitor(debug, bypass_behavior));
	if (pubsubstate==nullptr)
	{
		return false;
	}
	/// Initialize ROS publishers
	port_update_constraints = nh->advertise<rei_monitoring_msgs::MotionDynamicConstraints>("motion_dynamic_constraints", 10);
	/// Initialize ROS subscribers
	port_vehicle_status = nh->subscribe("vehicle_status", 10, &InterfaceRos_MotionConstraintMonitor::cbPort_vehicle_status, this);
	return true;
}



void InterfaceRos_MotionConstraintMonitor::cbPort_vehicle_status(const autoware_msgs::VehicleStatus::ConstPtr& msg)
{
	pubsubstate->msg_port_vehicle_status = *msg;
	executeUpdateconstraints(msg);
	
}

void InterfaceRos_MotionConstraintMonitor::publishMotion_dynamic_constraints()
{
	port_update_constraints.publish(pubsubstate->msg_port_update_constraints);
}

}
