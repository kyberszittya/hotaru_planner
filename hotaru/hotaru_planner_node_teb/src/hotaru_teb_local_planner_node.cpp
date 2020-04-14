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
	std::shared_ptr<ros::NodeHandle> nh = std::make_shared<ros::NodeHandle>();
	hotaru::HotaruTebLocalPlannerNode planner_node(nh, "base_link", "map", true);
	try
	{
		planner_node.initialize(true);
		ros::AsyncSpinner spinner(8);
		spinner.start();
		ros::waitForShutdown();
		spinner.stop();
		return 0;
	}
	catch(rei::ExceptionNodePreInitialization &e)
	{
		ROS_FATAL_STREAM("Unable to initialize HOTARU TEB LOCAL PLANNER" << e.what());
	}
	catch(rei::ExceptionNodeAssignSyncGuards &e)
	{
		ROS_FATAL_STREAM("Unable to initialize HOTARU TEB LOCAL PLANNER" << e.what());
	}
	catch(rei::ExceptionNodeMiddleWare &e)
	{
		ROS_FATAL_STREAM("Unable to initialize HOTARU TEB LOCAL PLANNER" << e.what());
	}
	return -1;
}
