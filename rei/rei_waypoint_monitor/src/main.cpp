/*
 * main.cpp
 *
 *  Created on: Feb 14, 2020
 *      Author: kyberszittya
 */

#include <ros/ros.h>

#include <std_msgs/Int32.h>
#include <geometry_msgs/PoseStamped.h>
#include <autoware_msgs/Lane.h>

#include <tf2_ros/transform_broadcaster.h>

namespace rei
{

double distanceBetweenTwoWaypoints(const geometry_msgs::PoseStamped& p0,
		const geometry_msgs::PoseStamped& p1)
{
	double dx = p0.pose.position.x - p1.pose.position.x;
	double dy = p0.pose.position.y - p1.pose.position.y;
	double dz = p0.pose.position.z - p1.pose.position.z;
	return std::sqrt(dx*dx + dy*dy + dz*dz);
}

class WaypointMonitor
{
protected:
	ros::NodeHandle nh;
	ros::Subscriber sub_current_pose;
	ros::Subscriber sub_current_velocity;
	ros::Subscriber sub_current_lane;
	ros::Publisher  pub_closest_waypoint;
	autoware_msgs::Lane current_lane;
	std_msgs::Int32 msg_closest_waypoint;
public:

	WaypointMonitor(const ros::NodeHandle& nh): nh(nh)
	{

	}

	void initRos()
	{
		sub_current_pose = nh.subscribe("current_pose", 10, &WaypointMonitor::cbCurrentPose, this);
		sub_current_lane = nh.subscribe("base_waypoints", 10, &WaypointMonitor::cbCurrentLane, this);
		pub_closest_waypoint = nh.advertise<std_msgs::Int32>("closest_waypoint", 10);
	}

	void cbCurrentPose(const geometry_msgs::PoseStamped::ConstPtr& msg)
	{
		// TODO: a little more intelligent search for closest point
		double min_dist = std::numeric_limits<double>::max();
		for (unsigned int i = 0; i < current_lane.waypoints.size(); i++)
		{
			double d = distanceBetweenTwoWaypoints(current_lane.waypoints[i].pose, *msg);
			if (d < min_dist)
			{
				min_dist = d;
				msg_closest_waypoint.data = i;
			}
		}
		pub_closest_waypoint.publish(msg_closest_waypoint);
	}

	void cbCurrentLane(const autoware_msgs::Lane::ConstPtr& msg)
	{
		current_lane = *msg;
	}

};

}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "waypoint_monitor");
	ros::NodeHandle nh;
	rei::WaypointMonitor wp_monitor(nh);
	wp_monitor.initRos();
	ROS_INFO("Starting waypoint monitor. Supporting the following information:");
	ROS_INFO("->  CLOSEST_WAYPOINT ");
	ros::spin();
	return 0;
}
