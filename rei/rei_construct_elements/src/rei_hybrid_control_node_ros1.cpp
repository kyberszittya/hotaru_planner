/*
 * rei_hybrid_control_node_ros1.cpp
 *
 *  Created on: Sep 9, 2020
 *      Author: kyberszittya
 */

#include <rei_signal_msgs/ReiNotificationSignal.h>
#include <rei_construct_elements/ros1/rei_hybrid_control_node_ros1.hpp>

namespace rei
{


namespace node
{

unsigned long RosClock::getCurrentTime()
{
	return ros::Time::now().toNSec();
}

HybridControlNodeRos1::HybridControlNodeRos1(std::shared_ptr<ros::NodeHandle> nh):
	HybridControlNode<unsigned long, RosClock, ros::Subscriber, ros::Publisher>(), nh(nh){}



bool HybridControlNodeRos1::init_control_signal_interface()
{
	// Publisher
	pub_controlevent = nh->advertise<rei_signal_msgs::ReiNotificationSignal>(
			hysm->getName()+"/control_events", 1);
	pub_transition_event = nh->advertise<rei_signal_msgs::ReiNotificationSignal>(
			hysm->getName()+"/transition_events", 1);
	pub_location_event = nh->advertise<rei_signal_msgs::ReiNotificationSignal>(
			hysm->getName()+"/location_events", 1);
	// Subscriber
	sub_controlsignal_pipeline = nh->subscribe(hysm->getName()+"/control_signal", 1,
			&HybridControlNodeRos1::subControlSignals, this);
	return true;
}

void HybridControlNodeRos1::subControlSignals(const rei_signal_msgs::ReiRuntimeControl::ConstPtr& msg)
{
	try
	{
		event_pipeline->propagateEvent(msg->event_name, msg->header.stamp.toNSec());
	}
	catch(rei::exceptions::UnknownEvent& e)
	{
		ROS_ERROR_STREAM("Event unknown: " << msg->event_name);
	}
}

void HybridControlNodeRos1::publishNotificationEvents()
{
	if (notification_context != nullptr)
	{
		NotificationEventPtrRos1 event = notification_context->popEvent();
		if (event != nullptr)
		{
			rei_signal_msgs::ReiNotificationSignal msg;
			msg.header.stamp.fromNSec(event->getTimestamp());
			msg.event_label = event->getEventName();
			msg.sm_name = hysm->getName();
			//msg.header.stamp = event->
			pub_transition_event.publish(msg);
		}
		NotificationEventPtrRos1 event_location = notification_context->popLocationEvent();
		if (event_location != nullptr)
		{
			rei_signal_msgs::ReiNotificationSignal msg;
			msg.header.stamp.fromNSec(event_location->getTimestamp());
			msg.event_label = event_location->getEventName();
			msg.sm_name = hysm->getName();
			pub_location_event.publish(msg);
		}
		NotificationEventPtrRos1 event_transition = notification_context->popTransitionEvent();
		if (event_transition != nullptr)
		{
			rei_signal_msgs::ReiNotificationSignal msg;
			msg.header.stamp.fromNSec(event_transition->getTimestamp());
			msg.event_label = event_transition->getEventName();
			msg.sm_name = hysm->getName();
			pub_transition_event.publish(msg);
		}
	}
	else throw std::runtime_error("Notification context is not assigned!");
}

void HybridControlNodeRos1::startTimer(const double timer_period)
{
	sm_timer = nh->createTimer(ros::Duration(timer_period), &HybridControlNodeRos1::opTimer, this);
}

void HybridControlNodeRos1::opTimer(const ros::TimerEvent& et)
{
	HybridStateStepResult res = HybridStateStepResult::TRANSITED;
	do
	{
		res = hysm->step();
		if (res==HybridStateStepResult::EMPTY_QUEUE)
		{
			break;
		}
	}while(res == HybridStateStepResult::TRANSITED);
}

} // namespace node


} // namespace rei

