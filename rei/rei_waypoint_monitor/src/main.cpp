/*
 * main.cpp
 *
 *  Created on: Feb 14, 2020
 *      Author: kyberszittya
 */

#include <ros/ros.h>

#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <hotaru_msgs/RefinedTrajectory.h>

#include <tf2_ros/transform_broadcaster.h>

#include <rei_common/geometric_utilities.hpp>

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

/**
 * @brief: Calculate the closest distance to a waypoint
 */
class WaypointMonitor
{
protected:
	ros::NodeHandle nh;
	ros::Subscriber sub_current_pose;
	ros::Subscriber sub_current_velocity;
	ros::Subscriber sub_current_lane;
	ros::Publisher  pub_closest_waypoint;
	ros::Publisher  pub_lateral_distance;
	hotaru_msgs::RefinedTrajectory current_trajectory;
	std_msgs::Int32 msg_closest_waypoint;
	std_msgs::Float64 msg_lateral_distance;
public:

	WaypointMonitor(const ros::NodeHandle& nh): nh(nh)
	{

	}

	void initRos()
	{
		sub_current_pose = nh.subscribe("current_pose", 10, &WaypointMonitor::cbCurrentPose, this);
		sub_current_lane = nh.subscribe("input_trajectory", 10, &WaypointMonitor::cbHighestLevelTrajectory, this);
		pub_closest_waypoint = nh.advertise<std_msgs::Int32>("closest_waypoint", 10);
		pub_lateral_distance = nh.advertise<std_msgs::Float64>("wp_abs_lateral_distance", 10);
	}

	void cbCurrentPose(const geometry_msgs::PoseStamped::ConstPtr& msg)
	{
		// TODO: a little more intelligent search for closest point
		double min_dist = std::numeric_limits<double>::max();
		int closest_waypoint = 0;
		double lateral_distance = 0.0;
		for (unsigned int i = 0; i < current_trajectory.waypoints.size(); i++)
		{
			double d = distanceBetweenTwoWaypoints(current_trajectory.waypoints[i].pose, *msg);
			if (d < min_dist)
			{
				min_dist = d;
				closest_waypoint = i;
			}
		}
		msg_closest_waypoint.data = closest_waypoint;
		// Don't care about extreme small lanes
		if (current_trajectory.waypoints.size() > 2)
		{
			// We are at the end of the lane
			if (closest_waypoint < current_trajectory.waypoints.size() - 1)
			{
				lateral_distance = rei::distanceToLine(
					current_trajectory.waypoints[closest_waypoint].pose.pose.position,
					current_trajectory.waypoints[closest_waypoint-1].pose.pose.position,
					msg->pose.position
				);
			}
			// We are at the beginning of everything
			else if (closest_waypoint == 0)
			{
				lateral_distance = rei::distanceToLine(
					current_trajectory.waypoints[0].pose.pose.position,
					current_trajectory.waypoints[1].pose.pose.position,
					msg->pose.position
				);
			}
			else
			{
				lateral_distance = rei::distanceToLine(
					current_trajectory.waypoints[current_trajectory.waypoints.size() - 2].pose.pose.position,
					current_trajectory.waypoints[current_trajectory.waypoints.size() - 1].pose.pose.position,
					msg->pose.position
				);
			}
			msg_lateral_distance.data = std::abs(lateral_distance);
		}
		pub_closest_waypoint.publish(msg_closest_waypoint);
		pub_lateral_distance.publish(msg_lateral_distance);
	}

	void cbHighestLevelTrajectory(const hotaru_msgs::RefinedTrajectory::ConstPtr& msg)
	{
		current_trajectory = *msg;
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
	ROS_INFO("->  CLOSEST WAYPOINT INDEX");
	ROS_INFO("->  LATERAL DISTANCE TO WAYPOINTS");
	ros::spin();
	return 0;
}
