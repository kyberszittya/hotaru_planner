/*
 * rei_hybrid_pipeline_ros1.hpp
 *
 *  Created on: Sep 9, 2020
 *      Author: kyberszittya
 */

#ifndef INCLUDE_REI_CONSTRUCT_ELEMENTS_ROS1_REI_HYBRID_PIPELINE_ROS1_HPP_
#define INCLUDE_REI_CONSTRUCT_ELEMENTS_ROS1_REI_HYBRID_PIPELINE_ROS1_HPP_


#include <rei_construct_elements/hybrid_dynamic_system/rei_hybrid_state_machine.hpp>
#include <rei_construct_elements/ros1/rei_hybrid_control_node_ros1.hpp>

namespace rei
{

namespace node
{

namespace ros1
{

typedef DiscreteEventPipeline<unsigned long, rei::node::RosClock> DiscreteEventPipelineRos1;
typedef DiscreteEventPipelinePtr<unsigned long, rei::node::RosClock> DiscreteEventPipelinePtrRos1;

} // namespace ros1

} // namespace node

} // namespace rei


#endif /* INCLUDE_REI_CONSTRUCT_ELEMENTS_ROS1_REI_HYBRID_PIPELINE_ROS1_HPP_ */
