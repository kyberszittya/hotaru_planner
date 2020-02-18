/*
 * test_simple_obstacle.cpp
 *
 *  Created on: Feb 12, 2020
 *      Author: kyberszittya
 */



#include <ros/ros.h>

#include <grid_map_msgs/GridMap.h>

#include <grid_map_cv/GridMapCvProcessing.hpp>

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_ros/PolygonRosConverter.hpp>

#include <iostream>
#include <autoware_msgs/Lane.h>
#include <autoware_msgs/LaneArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include "test_common.hpp"

constexpr int N = 40;
constexpr double XGoal = 25.0;

int main(int argc, char ** argv)
{
	grid_map::GridMap map({"elevation"});
	map.setFrameId("map");
	map["elevation"].setZero();
	grid_map::Position start_pos(8.0, 0.0);
	map.setGeometry(grid_map::Length(20.0, 10.0), 0.5, start_pos);
	ros::init(argc, argv, "simple_rect_demo");
	ros::NodeHandle nh;
	ros::Publisher publisher = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
	//ros::Publisher publisher_pose = nh.advertise<geometry_msgs::PoseStamped>("current_pose", 1, true);
	ros::Publisher publisher_goal = nh.advertise<geometry_msgs::PoseStamped>("current_goal", 1, true);
	//ros::Publisher publisher_velocity = nh.advertise<geometry_msgs::TwistStamped>("current_velocity", 1, true);
	ros::Publisher publisher_lane_array = nh.advertise<autoware_msgs::LaneArray>("lane_waypoints_array", 1, true);

	ros::Rate rate(10.0);
	hotaru::PlannerKinematicState state;
	state.pose.header.frame_id = "map";
	state.pose.pose.position.x = 0.0;
	state.pose.pose.position.y = 0.0;
	state.pose.pose.position.z = 0.0;
	//
	state.pose.pose.orientation.x = 0.0;
	state.pose.pose.orientation.y = 0.0;
	state.pose.pose.orientation.z = 0.0;
	state.pose.pose.orientation.w = 1.0;
	// Setup goal
	state.goal.header.frame_id = "map";
	state.goal.pose.position.x = 12.0;
	state.goal.pose.position.y = 2.0;
	state.goal.pose.orientation.w = 1.0;
	// Base waypoints
	autoware_msgs::LaneArray lane_array;
	lane_array.id = 0;
	autoware_msgs::Lane l;
	l.header.frame_id = "map";
	l.header.stamp = ros::Time::now();
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
	lane_array.lanes.push_back(l);
	while (nh.ok()){
		ros::Time time = ros::Time::now();

		grid_map::GridMapRosConverter::toMessage(map, message);
		publisher.publish(message);

		//publisher_pose.publish(state.pose);
		publisher_goal.publish(state.goal);
		publisher_lane_array.publish(lane_array);
		rate.sleep();

	}

	return 0;
}
