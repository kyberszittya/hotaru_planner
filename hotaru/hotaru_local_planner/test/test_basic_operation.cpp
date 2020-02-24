/*
 * test_basic_operation.cpp
 *
 *  Created on: Feb 23, 2020
 *      Author: kyberszittya
 */

#include <ros/ros.h>
#include <gtest/gtest.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <autoware_msgs/Lane.h>
#include <rei_monitoring_msgs/ReiStateMachineTransitionSignal.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_ros/PolygonRosConverter.hpp>

// SKULL ERROR DISMISSED: this would not work any other way
std::shared_ptr<ros::NodeHandle> nh;

constexpr int N = 40;
constexpr double XGoal = 25.0;

TEST(HotaruLocalPlannerTest, BasicLocalPlannerTest)
{
	ros::Publisher publisher_grid_map = nh->advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
	ros::Publisher publisher_pose = nh->advertise<geometry_msgs::PoseStamped>("current_pose", 1, true);
	ros::Publisher publisher_velocity = nh->advertise<geometry_msgs::TwistStamped>("current_velocity", 1, true);
	ros::Publisher publisher_base_waypoints = nh->advertise<autoware_msgs::Lane>("base_waypoints", 1, true);

	ros::Rate rate(10.0);
	geometry_msgs::PoseStamped pose;
	pose.header.frame_id = "map";
	pose.pose.position.x = 0.0;
	pose.pose.position.y = 0.0;
	pose.pose.position.z = 0.0;
	//
	pose.pose.orientation.x = 0.0;
	pose.pose.orientation.y = 0.0;
	pose.pose.orientation.z = 0.0;
	pose.pose.orientation.w = 1.0;
	// Setup velocity
	geometry_msgs::TwistStamped velocity;
	velocity.header.frame_id = "map";
	velocity.twist.linear.x = 10;
	velocity.twist.linear.y = 0;
	velocity.twist.linear.z = 0;
	velocity.twist.angular.x = 0;
	velocity.twist.angular.y = 0;
	velocity.twist.angular.z = 0;

	// Base waypoints
	autoware_msgs::Lane l;
	l.header.frame_id = "map";

	for (int i = 0; i <= N; i++)
	{
		autoware_msgs::Waypoint wp1;
		wp1.pose.pose.position.x = i*(XGoal/N);
		wp1.pose.pose.position.y = 0.0;
		wp1.pose.pose.position.z = 0.0;
		wp1.pose.pose.orientation.w = 1.0;
		wp1.twist.twist.linear.x = 10.0;
		l.waypoints.push_back(std::move(wp1));
	}

	grid_map::GridMap map({"elevation"});
	map.setFrameId("map");
	map["elevation"].setZero();
	grid_map::Position start_pos(8.0, 0.0);
	map.setGeometry(grid_map::Length(20.0, 10.0), 0.5, start_pos);
	for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it)
	{
		grid_map::Position position;
		map.getPosition(*it, position);
		map.at("elevation", *it) = 0.0;
	}
	grid_map::Position pos_obstacle(10.0, 0.0);
	grid_map::Index index_obstacle;
	map.getIndex(pos_obstacle, index_obstacle);
	grid_map::Position rel_obstacle_pose;
	map.getPosition(index_obstacle, rel_obstacle_pose);
	map.atPosition("elevation", rel_obstacle_pose) = 1.0;
	grid_map_msgs::GridMap message;
	// Update some messages
	ros::Rate r(40);
	for (unsigned int i = 0; i < 100; i++)
	{
		ros::Time time = ros::Time::now();
		pose.header.stamp = time;
		velocity.header.stamp = time;
		l.header.stamp = time;
		grid_map::GridMapRosConverter::toMessage(map, message);
		publisher_pose.publish(pose);
		publisher_velocity.publish(velocity);
		publisher_base_waypoints.publish(l);
		publisher_grid_map.publish(message);
		r.sleep();
	}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "local_planner_tester");
  nh.reset(new ros::NodeHandle);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

