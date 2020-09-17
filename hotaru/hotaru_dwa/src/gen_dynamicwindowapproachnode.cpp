#include <hotaru_dwa/gen_dynamicwindowapproachnode.hpp>

namespace hotaru {

InterfaceRos_DynamicWindowApproachNode::~InterfaceRos_DynamicWindowApproachNode() {}

bool InterfaceRos_DynamicWindowApproachNode::initTimeoutStateMachine()
{
	// Sync state machine initialization
	return true;
}

bool InterfaceRos_DynamicWindowApproachNode::initMiddleware(const bool debug, const bool bypass_behavior)
{
	/// Call superinterface
	InterfaceRos_DynamicWindowApproachNode::initMiddleware(debug, bypass_behavior);
	/// Initialize internal pubsub state
	pubsubstate = std::unique_ptr<StateDynamicWindowApproachNode>(new StateDynamicWindowApproachNode(debug, bypass_behavior));
	if (pubsubstate==nullptr)
	{
		return false;
	}
	/// Initialize ROS publishers
	port_output_dwa_optim = nh->advertise<grid_map_msgs::GridMap>("optim_boundary", 10);
	/// Initialize ROS subscribers
	port_input_dynamic_window = nh->subscribe("dynamic_window", 10, &InterfaceRos_DynamicWindowApproachNode::cbPort_input_dynamic_window, this);
	return true;
}



void InterfaceRos_DynamicWindowApproachNode::cbPort_input_dynamic_window(const grid_map_msgs::GridMap::ConstPtr& msg)
{
	pubsubstate->msg_port_input_dynamic_window = *msg;
	
}

void InterfaceRos_DynamicWindowApproachNode::publishOptim_boundary()
{
	port_output_dwa_optim.publish(pubsubstate->msg_port_output_dwa_optim);
}

}
