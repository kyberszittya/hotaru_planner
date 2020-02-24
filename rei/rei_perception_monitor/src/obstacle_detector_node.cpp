/*
 * obstacle_detector_node.cpp
 *
 *  Created on: Feb 12, 2020
 *      Author: kyberszittya
 */

#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>

#include <geometry_msgs/PoseStamped.h>

#include <visualization_msgs/MarkerArray.h>


namespace rei
{

class ObstacleGridMapMonitor
{
private:

protected:
	geometry_msgs::PoseStamped current_pose;
	ros::NodeHandle nh;
	ros::Subscriber sub_grid_map;
	ros::Subscriber sub_current_pose;
	ros::Publisher pub_detected_obstacles;
	grid_map::GridMap global_map;
	// Detect polygon obstacles
	visualization_msgs::MarkerArray obstacles;
public:
	ObstacleGridMapMonitor(ros::NodeHandle& nh): nh(nh){}

	bool init()
	{
		pub_detected_obstacles = nh.advertise<visualization_msgs::MarkerArray>("/filtered_obstacles", 10);
		sub_current_pose = nh.subscribe("current_pose", 10, &ObstacleGridMapMonitor::cbCurrentPose, this);
		sub_grid_map = nh.subscribe("grid_map", 1, &ObstacleGridMapMonitor::subGridMap, this);

		return true;
	}

	void cbCurrentPose(const geometry_msgs::PoseStamped::ConstPtr& msg)
	{
		current_pose = *msg;
	}

	void subGridMap(const grid_map_msgs::GridMap::ConstPtr& msg)
	{
		grid_map::GridMapRosConverter::fromMessage(*msg, global_map);
		grid_map::Position pos_current(
						current_pose.pose.position.x,
						current_pose.pose.position.y);
		obstacles.markers.clear();
		auto t = ros::Time::now();
		for (grid_map::GridMapIterator iterator(global_map);
							!iterator.isPastEnd(); ++iterator)
		{
			if (global_map.at("elevation", *iterator) > 0.5)
			{
				visualization_msgs::Marker m;
				m.header.frame_id = "map";
				m.header.stamp = t;
				m.type = visualization_msgs::Marker::SPHERE;
				grid_map::Position3 pos;
				global_map.getPosition3("elevation", *(iterator), pos);
				m.pose.position.x = pos[0];
				m.pose.position.y = pos[1];
				m.pose.orientation.w = 1.0;
				m.scale.x = global_map.getResolution()*2.0;
				m.scale.y = global_map.getResolution()*2.0;
				m.scale.z = global_map.getResolution()*2.0;
				// Set color
				m.color.a = 1.0;
				m.color.r = 0.13;
				m.color.g = 0.7;
				m.color.b = 0.8;
				obstacles.markers.push_back(std::move(m));
			}
		}
		pub_detected_obstacles.publish(obstacles);
	}
};

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "rei_obstacle_monitor");
	ros::NodeHandle nh;
	rei::ObstacleGridMapMonitor obstacle_monitor(nh);
	if (obstacle_monitor.init())
	{
		ROS_INFO("Successfully started obstacle detection");
		ros::spin();
		return 0;
	}
	return -1;
}
