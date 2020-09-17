#ifndef DYNAMICWINDOWAPPROACHNODE_HEADER_HPP
#define DYNAMICWINDOWAPPROACHNODE_HEADER_HPP

#include <ros/ros.h>
/// ROS msgs
#include <grid_map_msgs/GridMap.h>
// State-machine node element
#include <rei_common/gen_node_elements/interface_simple_ros_node.hpp>

#include <memory>

namespace hotaru {

struct StateDynamicWindowApproachNode
{
	// Flags
	const bool debug;				///< Publish debug parameters
	const bool bypass_behavior;     ///< Bypass behavioral state machines
	// ROS input messages
	grid_map_msgs::GridMap msg_port_input_dynamic_window; ///< port_input_dynamic_window store to grid_map_msgs::GridMap
	/// ROS output messages
	grid_map_msgs::GridMap msg_port_output_dwa_optim; ///< port_output_dwa_optim store to grid_map_msgs::GridMap
	
	StateDynamicWindowApproachNode(const bool debug, const bool bypass_behavior): debug(debug), bypass_behavior(bypass_behavior) {}
};

/**
 *
 * @attribute port_input_dynamic_window: subscribes to topic dynamic_window 
 * @attribute port_output_dwa_optim: publishes to topic optim_boundary
 */
class InterfaceRos_DynamicWindowApproachNode: public hotaru::InterfaceRos_AbstractHotaruPlannerNode
{
private:
protected:
	/// ROS utils
	std::shared_ptr<ros::NodeHandle> private_nh;
	std::shared_ptr<ros::NodeHandle> nh;
	/// ROS Subscribers
	ros::Subscriber port_input_dynamic_window; ///< port_input_dynamic_window subscriber to grid_map_msgs::GridMap
	/// ROS Publishers
	ros::Publisher port_output_dwa_optim; ///< port_output_dwa_optim publisher to grid_map_msgs::GridMap
	std::unique_ptr<StateDynamicWindowApproachNode> pubsubstate;
	// State machines
public:
	InterfaceRos_DynamicWindowApproachNode(std::shared_ptr<ros::NodeHandle> private_nh, std::shared_ptr<ros::NodeHandle> nh): private_nh(private_nh), nh(nh) {}
	
	virtual ~InterfaceRos_DynamicWindowApproachNode() = 0;
	
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
	 * Callback method for dynamic_window
	 */
	void cbPort_input_dynamic_window(const grid_map_msgs::GridMap::ConstPtr& msg); ///< port_input_dynamic_window subscriber to grid_map_msgs::GridMap
	
	/**
	 * Publish method to publish message to optim_boundary
	 */
	void publishOptim_boundary();
};

}

#endif
