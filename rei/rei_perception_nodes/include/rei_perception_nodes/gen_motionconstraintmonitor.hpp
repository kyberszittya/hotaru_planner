#ifndef MOTIONCONSTRAINTMONITOR_HEADER_HPP
#define MOTIONCONSTRAINTMONITOR_HEADER_HPP

#include <ros/ros.h>
/// ROS msgs
#include <autoware_msgs/VehicleStatus.h>
#include <rei_monitoring_msgs/MotionDynamicConstraints.h>
// State-machine node element
#include <rei_common/gen_node_elements/interface_simple_ros_node.hpp>

#include <memory>

namespace rei {

struct StateMotionConstraintMonitor
{
	// Flags
	const bool debug;				///< Publish debug parameters
	const bool bypass_behavior;     ///< Bypass behavioral state machines
	// ROS input messages
	autoware_msgs::VehicleStatus msg_port_vehicle_status; ///< port_vehicle_status store to autoware_msgs::VehicleStatus
	/// ROS output messages
	rei_monitoring_msgs::MotionDynamicConstraints msg_port_update_constraints; ///< port_update_constraints store to rei_monitoring_msgs::MotionDynamicConstraints
	
	StateMotionConstraintMonitor(const bool debug, const bool bypass_behavior): debug(debug), bypass_behavior(bypass_behavior) {}
};

/**
 *
 * @attribute port_vehicle_status: subscribes to topic vehicle_status 
 * @attribute port_update_constraints: publishes to topic motion_dynamic_constraints
 */
class InterfaceRos_MotionConstraintMonitor: public rei::Interface_SimpleRosNode
{
private:
protected:
	/// ROS utils
	std::shared_ptr<ros::NodeHandle> private_nh;
	std::shared_ptr<ros::NodeHandle> nh;
	/// ROS Subscribers
	ros::Subscriber port_vehicle_status; ///< port_vehicle_status subscriber to autoware_msgs::VehicleStatus
	/// ROS Publishers
	ros::Publisher port_update_constraints; ///< port_update_constraints publisher to rei_monitoring_msgs::MotionDynamicConstraints
	std::unique_ptr<StateMotionConstraintMonitor> pubsubstate;
	// State machines
public:
	InterfaceRos_MotionConstraintMonitor(std::shared_ptr<ros::NodeHandle> private_nh, std::shared_ptr<ros::NodeHandle> nh): private_nh(private_nh), nh(nh) {}
	
	virtual ~InterfaceRos_MotionConstraintMonitor() = 0;
	
	/*
	 * Override steps to initialize
	 *     STEPS:
	 *           1. Initialize descendant specific functionalities before middleware functionalities (initPre())
	 *           2. Initialize timeout state machine (initTimeoutStateMachine())
	 *           3. Assign guard related to timeout functions (assigSyncGuards())
	 *           4. Initialize middleware functionalities
	 *           5. Initialize descendant node-specific functionalities
	 */
	/*
	 * @brief: Initialize node pre
	 * @returns: Initialization successful
	 */
	virtual bool initPre() = 0;
	/*
	 * @brief: Initialize timeout statemachine
	 */
	virtual bool initTimeoutStateMachine() override;
	/*
	 * @brief: Assign sync guards
	 */
	virtual bool assignSyncGuards() = 0;
	
	/*
	 * @brief: initialize middleware
	 * @param debug: defines whether the debug information should be provided or not.
	 */
	virtual bool initMiddleware(const bool debug, const bool bypass_behavior) override;
	
	/*
	 * @brief: post initialize
	 */
	virtual bool initPost() = 0;
	
	/**
	 * Callback method for vehicle_status
	 */
	void cbPort_vehicle_status(const autoware_msgs::VehicleStatus::ConstPtr& msg); ///< port_vehicle_status subscriber to autoware_msgs::VehicleStatus
	virtual void executeUpdateconstraints(const autoware_msgs::VehicleStatus::ConstPtr& msg) = 0;
	
	/**
	 * Publish method to publish message to motion_dynamic_constraints
	 */
	void publishMotion_dynamic_constraints();
};

}

#endif
