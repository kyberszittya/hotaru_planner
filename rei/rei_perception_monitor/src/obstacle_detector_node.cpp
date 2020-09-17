/*
 * obstacle_detector_node.cpp
 *
 *  Created on: Feb 12, 2020
 *      Author: kyberszittya
 */



#include <rei_perception_monitor/object_waypoint_detector.hpp>



int main(int argc, char** argv)
{
	ros::init(argc, argv, "rei_obstacle_monitor");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");
	// Read frames

	std::string global_frame;
	if (!private_nh.getParam("global_frame", global_frame))
	{
		global_frame = rei::DEFAULT_GLOBAL_FRAME;
		ROS_WARN_STREAM("No global frame has been specified, using default: " << global_frame);
	}
	std::string local_frame;
	if (!private_nh.getParam("local_frame", local_frame))
	{
		local_frame = rei::DEFAULT_LOCAL_FRAME;
		ROS_WARN_STREAM("No global frame has been specified, using default: " << local_frame);
	}
	std::string sensor_frame;
	if (!private_nh.getParam("sensor_frame", sensor_frame))
	{
		sensor_frame = rei::DEFAULT_SENSOR_FRAME;
		ROS_WARN_STREAM("No object-detecting sensor frame has been specified, using default: " << sensor_frame);
	}
	rei::ObstacleGridMapMonitor obstacle_monitor(global_frame, local_frame, sensor_frame, nh);

	if (obstacle_monitor.init())
	{
		ROS_INFO("Successfully started obstacle detection");
		ros::spin();
		return 0;
	}
	return -1;
}
