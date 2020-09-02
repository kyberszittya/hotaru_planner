/*
 * rei_hybrid_control_node_ros1.hpp
 *
 *  Created on: Sep 2, 2020
 *      Author: kyberszittya
 */

#ifndef INCLUDE_REI_CONSTRUCT_ELEMENTS_REI_HYBRID_CONTROL_NODE_ROS1_HPP_
#define INCLUDE_REI_CONSTRUCT_ELEMENTS_REI_HYBRID_CONTROL_NODE_ROS1_HPP_


#include <ros/ros.h>

#include "rei_hybrid_control_node.hpp"

namespace rei
{

namespace node
{

class HybridControlNodeRos1: public HybridControlNode<ros::Subscriber, ros::Publisher>
{
private:
public:
	virtual ~HybridControlNodeRos1() = 0;

	/**
	 * @brief: We need an ROS 1 version of this.
	 */
	virtual bool init_control_signal_interface() override;
};

} // namespace node

} // namespace rei


#endif /* INCLUDE_REI_CONSTRUCT_ELEMENTS_REI_HYBRID_CONTROL_NODE_ROS1_HPP_ */
