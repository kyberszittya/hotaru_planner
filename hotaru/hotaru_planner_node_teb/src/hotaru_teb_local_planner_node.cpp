/*
 * hotaru_teb_local_planner_node.cpp
 *
 *  Created on: Apr 1, 2020
 *      Author: kyberszittya
 */
#include <ros/ros.h>

#include <hotaru_planner_node_teb/hotaru_teb_local_planner.hpp>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "hotaru_teb_planner_node");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");
	hotaru::HotaruTebLocalPlannerNode planner_node(nh, private_nh);
	if (planner_node.init())
	{
		ros::spin();
		return -1;
	}
	ROS_ERROR("Unable to initialize HOTARU TEB LOCAL PLANNER");
	return -1;
}
