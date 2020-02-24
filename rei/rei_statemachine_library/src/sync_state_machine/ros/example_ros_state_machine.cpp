/*
 * example_ros_state_machine.cpp
 *
 *  Created on: Feb 21, 2020
 *      Author: kyberszittya
 */


#include <rei_statemachine_library/ros/ros_sync_state_machine.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Header.h>

class SimpleSubscriber
{
private:
	std::shared_ptr<rei::RosSyncStateMachine> sm;
	std::shared_ptr<ros::NodeHandle> nh;
	ros::Subscriber sub_random_vehicle_command;
public:
	SimpleSubscriber(std::shared_ptr<ros::NodeHandle> nh,
			std::shared_ptr<rei::RosSyncStateMachine> sm): nh(nh), sm(sm)
	{

	}

	void init()
	{
		sub_random_vehicle_command = nh->subscribe(
				"/random_vehicle_command", 10,
				&SimpleSubscriber::cbRandomVehicleCommand, this);
	}


	void cbRandomVehicleCommand(const geometry_msgs::PoseStamped::ConstPtr& msg)
	{
		sm->stepMessageTopic("/random_vehicle_command", msg->header);
	}
};

int main(int argc, char** argv)
{
	using namespace rei;
	ros::init(argc, argv, "simple_sync_state");
	std::shared_ptr<ros::NodeHandle> nh(new ros::NodeHandle);
	std::shared_ptr<RosSyncStateMachine> sm(new RosSyncStateMachine(
			nh, "simple_sync_state"));
	if (sm->initialize())
	{
		SimpleSubscriber simple_sub(nh, sm);
		sm->addTopicGuard("/random_vehicle_command", 1.5);
		ROS_INFO("Initialized simple sync state machine");
		simple_sub.init();
		ros::Rate r(20.0);
		while (ros::ok())
		{
			ros::spinOnce();
			r.sleep();
		}
	}
	else
	{
		ROS_ERROR("Unable to initialize state machine");
		return -1;
	}
	return 0;
}

