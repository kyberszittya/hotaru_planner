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

	/**
		 * @fn unsigned long getCurrentTime()
	 * @brief Return current ROS time
	 *
	 * @pre
	 * @post
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
// Redefine ROS 1 location event
typedef LocationNotificationEvent<unsigned long> LocationNotificationEventRos1;
typedef LocationNotificationEventPtr<unsigned long> LocationNotificationEventPtrRos1;
// Redefine ROS 1 transition event
typedef TransitionNotificationEvent<unsigned long> TransitionNotificationEventRos1;
typedef TransitionNotificationEventPtr<unsigned long> TransitionNotificationEventPtrRos1;
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
		 * @fn bool init_control_signal_interface()
	 * @brief Initialize control signal interface for ROS 1.
	 *
	 * @pre
	 * @post
	 * @return
	 */
	virtual bool init_control_signal_interface() override;

	void subControlSignals(const rei_signal_msgs::ReiRuntimeControl::ConstPtr& msg);

	void publishNotificationEvents();


	/**
		 * @fn void startTimer(const double)
	 * @brief Start timer thread processing events and stepping.
	 *
	 * @pre
	 * @post
	 * @param timer_period
	 */
	void startTimer(const double timer_period);


	/**
		 * @fn void opTimer(const ros::TimerEvent&)
	 * @brief Timer
	 *
	 * @pre
	 * @post
	 * @param et ROS timer event
	 */
	void opTimer(const ros::TimerEvent& et);
};

} // namespace node

} // namespace rei


#endif /* INCLUDE_REI_CONSTRUCT_ELEMENTS_REI_HYBRID_CONTROL_NODE_ROS1_HPP_ */
