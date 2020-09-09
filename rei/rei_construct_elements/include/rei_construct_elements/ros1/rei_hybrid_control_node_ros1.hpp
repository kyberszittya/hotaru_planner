/*
 * rei_hybrid_control_node_ros1.hpp
 *
 *  Created on: Sep 2, 2020
 *      Author: kyberszittya
 */

#ifndef INCLUDE_REI_CONSTRUCT_ELEMENTS_REI_HYBRID_CONTROL_NODE_ROS1_HPP_
#define INCLUDE_REI_CONSTRUCT_ELEMENTS_REI_HYBRID_CONTROL_NODE_ROS1_HPP_


#include <ros/ros.h>

#include <rei_construct_elements/rei_hybrid_control_node.hpp>
#include <rei_construct_elements/rei_control_system.hpp>
#include <rei_construct_elements/hybrid_dynamic_system/rei_hybrid_state_machine_factory.hpp>

#include <rei_signal_msgs/ReiRuntimeControl.h>

namespace rei
{

namespace node
{

class RosClock: public rei::util::ClockInterface<unsigned long>
{
public:

	/*
	 * Return current ROS time
	 */
	virtual unsigned long getCurrentTime() override;
};

// Redefine event
typedef NotificationEventPtr<unsigned long> NotificationEventPtrRos1;
// Redefine factory for state machine
typedef HybridStateMachineFactory<unsigned long, RosClock> HybridStateMachineFactoryRos1;
// Redefine ROS 1 hybrid state machine
typedef HybridStateMachine<unsigned long, RosClock> HybridStateMachineRos1;
typedef HybridStateMachinePtr<unsigned long, RosClock> HybridStateMachinePtrRos1;
// Redefine ROS 1 control node
typedef hybridsystem::HybridControl<unsigned long, RosClock> HybridControlRos1;
typedef std::shared_ptr<hybridsystem::HybridControl<unsigned long, RosClock>> HybridControlPtrRos1;
// Redefine ROS 1 notification context
typedef NotificationContext<unsigned long> NotificationContextRos1;
typedef NotificationContextPtr<unsigned long> NotificationContextPtrRos1;

class HybridControlNodeRos1: public HybridControlNode<unsigned long, RosClock, ros::Subscriber, ros::Publisher>
{
private:
	std::shared_ptr<ros::NodeHandle> nh;
	ros::Timer sm_timer;
public:
	HybridControlNodeRos1(std::shared_ptr<ros::NodeHandle> nh);


	/**
	 * @brief: We need an ROS 1 version of this.
	 */
	virtual bool init_control_signal_interface() override;

	void subControlSignals(const rei_signal_msgs::ReiRuntimeControl::ConstPtr& msg);

	void publishNotificationEvents();

	/*
	 * @brief: Start timer
	 */
	void startTimer(const double timer_period);

	/*
	 * @brief: Timer operation (stepping state machine)
	 */
	void opTimer(const ros::TimerEvent& et);
};

} // namespace node

} // namespace rei


#endif /* INCLUDE_REI_CONSTRUCT_ELEMENTS_REI_HYBRID_CONTROL_NODE_ROS1_HPP_ */
