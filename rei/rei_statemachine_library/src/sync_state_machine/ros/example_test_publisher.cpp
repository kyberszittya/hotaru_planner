/*
 * example_test_publisher.cpp
 *
 *  Created on: Feb 21, 2020
 *      Author: kyberszittya
 */

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>


int main(int argc, char** argv)
{
	ros::init(argc, argv, "simple_geom_publisher");
	ros::NodeHandle nh;
	ros::Publisher pub_vehicle_cmd;
	pub_vehicle_cmd = nh.advertise<geometry_msgs::PoseStamped>(
			"/random_vehicle_command", 10);
	ros::Rate r(10);
	geometry_msgs::PoseStamped msg_pose;
	while(ros::ok())
	{
		msg_pose.header.stamp = ros::Time::now();
		pub_vehicle_cmd.publish(msg_pose);
		r.sleep();
	}
	return 0;
}
