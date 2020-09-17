#ifndef HOTARUTEBLOCALPLANNERNODE_HEADER_HPP
#define HOTARUTEBLOCALPLANNERNODE_HEADER_HPP

#include <ros/ros.h>
/// ROS msgs
// State-machine node element
#include <rei_common/gen_node_elements/interface_simple_ros_node.hpp>

#include <memory>

namespace hotaru {

struct StateHotaruTebLocalPlannerNode
{
	// Flags
	const bool debug;				///< Publish debug parameters
	const bool bypass_behavior;     ///< Bypass behavioral state machines
	// ROS input messages
	/// ROS output messages
	
	StateHotaruTebLocalPlannerNode(const bool debug, const bool bypass_behavior): debug(debug), bypass_behavior(bypass_behavior) {}
};

/**
 *
 */
class InterfaceRos_HotaruTebLocalPlannerNode: public rei::Interface_SimpleRosNode
{
private:
protected:
	/// ROS utils
	std::shared_ptr<ros::NodeHandle> private_nh;
	std::shared_ptr<ros::NodeHandle> nh;
	/// ROS Subscribers
	/// ROS Publishers
	std::unique_ptr<StateHotaruTebLocalPlannerNode> pubsubstate;
	// State machines
	// Set ALL STATES CB
	virtual void setSyncStateMachineCallbacks() = 0;
public:
	InterfaceRos_HotaruTebLocalPlannerNode(std::shared_ptr<ros::NodeHandle> private_nh, std::shared_ptr<ros::NodeHandle> nh): private_nh(private_nh), nh(nh) {}
	
	virtual ~InterfaceRos_HotaruTebLocalPlannerNode() = 0;
	
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
	
	
};

}

#endif
