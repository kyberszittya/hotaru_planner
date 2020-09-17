/*
 * hotaru_teb_local_planner_node.cpp
 *
 *  Created on: Apr 1, 2020
 *      Author: kyberszittya
 */
#include <ros/ros.h>

#include <hotaru_planner_node_teb/hotaruteblocalplannernode.hpp>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "hotaru_teb_planner_node");
	std::shared_ptr<ros::NodeHandle> nh = std::make_shared<ros::NodeHandle>();
	std::shared_ptr<ros::NodeHandle> private_nh = std::make_shared<ros::NodeHandle>("~");
	// Read whether to debug
	bool is_debug;
	if (!private_nh->getParam("debug", is_debug))
	{
		ROS_INFO("Not publishing debug information");
		is_debug = false;
	}
	//
	bool is_bypass_behavior;
	if (!private_nh->getParam("bypass_behavior", is_bypass_behavior))
	{
		is_bypass_behavior = false;
	}
	if (is_bypass_behavior)
	{
		ROS_WARN("Local planner hybrid behavior bypassed");
	}
	//
	hotaru::HotaruTebLocalPlannerNode planner_node(private_nh, nh, "base_link", "map");
	try
	{
		planner_node.initialize(is_debug, is_bypass_behavior);
		//planner_node.config(private_nh);
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
