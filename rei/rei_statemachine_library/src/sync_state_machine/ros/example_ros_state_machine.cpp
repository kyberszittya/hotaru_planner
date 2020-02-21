/*
 * example_ros_state_machine.cpp
 *
 *  Created on: Feb 21, 2020
 *      Author: kyberszittya
 */


#include <rei_statemachine_library/ros/ros_sync_state_machine.hpp>
#include <std_msgs/Float64.h>
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


	void cbRandomVehicleCommand(const std_msgs::Float64::ConstPtr& msg)
	{
		std_msgs::Header h;
		h.stamp = ros::Time::now();
		sm->stepMessageTopic("/random_vehicle_command", h);

	}
};

int main(int argc, char** argv)
{
	using namespace rei;
	ros::init(argc, argv, "simple_sync_state");
	std::shared_ptr<ros::NodeHandle> nh(new ros::NodeHandle);
	std::shared_ptr<RosSyncStateMachine> sm(new RosSyncStateMachine(
			nh, "simple_sync_state"));
	SimpleSubscriber simple_sub(nh, sm);
	sm->addTopicGuard("/random_vehicle_command", 10);
	sm->initialize();

	return 0;
}

