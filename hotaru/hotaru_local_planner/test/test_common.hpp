/*
 * test_common.hpp
 *
 *  Created on: Feb 25, 2020
 *      Author: kyberszittya
 */

#ifndef TEST_TEST_COMMON_HPP_
#define TEST_TEST_COMMON_HPP_

#include <ros/ros.h>

#include <gtest/gtest.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <autoware_msgs/Lane.h>
#include <rei_monitoring_msgs/ReiStateMachineTransitionSignal.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_ros/PolygonRosConverter.hpp>
#include <tf2_ros/transform_broadcaster.h>

std::shared_ptr<ros::NodeHandle> nh;

struct PlanarOffset
{
	double x;
	double y;
};


constexpr double XGoal = 25.0;

constexpr int hzrate = 40;

struct RosInterfaceTest
{
	ros::Publisher publisher_grid_map = nh->advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
	ros::Publisher publisher_pose = nh->advertise<geometry_msgs::PoseStamped>("current_pose", 1, true);
	ros::Publisher publisher_velocity = nh->advertise<geometry_msgs::TwistStamped>("current_velocity", 1, true);
	ros::Publisher publisher_base_waypoints = nh->advertise<autoware_msgs::Lane>("base_waypoints", 1, true);
	tf2_ros::TransformBroadcaster br;
};

struct TestDynamics
{
	geometry_msgs::TwistStamped velocity;
	geometry_msgs::PoseStamped pose;
	geometry_msgs::TransformStamped transformStamped;
	autoware_msgs::Lane l;
};

void setupTestCase(RosInterfaceTest& rostestinterface,
		TestDynamics& dyn, const PlanarOffset& pos_offset,
		unsigned int N,
		double length = XGoal)
{
	rostestinterface.publisher_grid_map = nh->advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
	rostestinterface.publisher_pose = nh->advertise<geometry_msgs::PoseStamped>("current_pose", 1, true);
	rostestinterface.publisher_velocity = nh->advertise<geometry_msgs::TwistStamped>("current_velocity", 1, true);
	rostestinterface.publisher_base_waypoints = nh->advertise<autoware_msgs::Lane>("base_waypoints", 1, true);

	dyn.pose.header.frame_id = "map";
	dyn.pose.pose.position.x = pos_offset.x;
	dyn.pose.pose.position.y = pos_offset.y;
	dyn.pose.pose.position.z = 0.0;
	//
	dyn.pose.pose.orientation.x = 0.0;
	dyn.pose.pose.orientation.y = 0.0;
	dyn.pose.pose.orientation.z = 0.0;
	dyn.pose.pose.orientation.w = 1.0;
	// Setup velocity

	dyn.velocity.header.frame_id = "base_link";
	dyn.velocity.twist.linear.x = 10;
	dyn.velocity.twist.linear.y = 0;
	dyn.velocity.twist.linear.z = 0;
	dyn.velocity.twist.angular.x = 0;
	dyn.velocity.twist.angular.y = 0;
	dyn.velocity.twist.angular.z = 0;
	// Setup transform


	dyn.transformStamped.header.frame_id = "map";
	dyn.transformStamped.child_frame_id = "base_link";
	dyn.transformStamped.transform.translation.x = pos_offset.x;
	dyn.transformStamped.transform.translation.y = pos_offset.y;
	dyn.transformStamped.transform.translation.z = 0.0;
	dyn.transformStamped.transform.rotation.w = 1.0;
	// Base waypoints

	dyn.l.header.frame_id = "map";
	for (int i = 0; i <= N; i++)
	{
		autoware_msgs::Waypoint wp1;
		wp1.pose.header.frame_id = "map";
		wp1.pose.pose.position.x = pos_offset.x + i*(length/N);
		wp1.pose.pose.position.y = pos_offset.y + 0.0;
		wp1.pose.pose.position.z = 0.0;
		wp1.pose.pose.orientation.w = 1.0;
		wp1.twist.twist.linear.x = 1.0;
		dyn.l.waypoints.push_back(std::move(wp1));
	}
}

void testPlanningScenario(const PlanarOffset& obstacle_offset,
		const PlanarOffset& pos_offset,
		unsigned int N,
		unsigned int T_test,
		double length = XGoal)
{
	TestDynamics dyn;
	RosInterfaceTest rostestinterface;
	setupTestCase(rostestinterface, dyn, pos_offset, N, length);

	grid_map::GridMap map({"elevation"});
	map.setFrameId("base_link");
	map["elevation"].setZero();
	grid_map::Position start_pos(8.0, 0.0);
	map.setGeometry(grid_map::Length(20.0, 10.0), 0.5, start_pos);
	for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it)
	{
		grid_map::Position position;
		map.getPosition(*it, position);
		map.at("elevation", *it) = 0.0;
	}
	grid_map::Position pos_obstacle(
			obstacle_offset.x,
			obstacle_offset.y);
	grid_map::Index index_obstacle;
	map.getIndex(pos_obstacle, index_obstacle);
	grid_map::Position rel_obstacle_pose;
	map.getPosition(index_obstacle, rel_obstacle_pose);
	map.atPosition("elevation", rel_obstacle_pose) = 1.0;
	grid_map_msgs::GridMap message;
	// Update some messages
	ros::Rate r(hzrate);
	for (unsigned int i = 0; i < hzrate*T_test; i++)
	{
		ros::Time time = ros::Time::now();
		dyn.transformStamped.header.stamp = time;
		dyn.pose.header.stamp = time;
		dyn.velocity.header.stamp = time;
		dyn.l.header.stamp = time;
		grid_map::GridMapRosConverter::toMessage(map, message);
		rostestinterface.publisher_pose.publish(dyn.pose);
		rostestinterface.publisher_velocity.publish(dyn.velocity);
		rostestinterface.publisher_base_waypoints.publish(dyn.l);
		rostestinterface.br.sendTransform(dyn.transformStamped);
		rostestinterface.publisher_grid_map.publish(message);
		//

		r.sleep();
	}
}

void testPlanningScenarioLinearMovingObstacle(
		const PlanarOffset& obstacle_offset_0,
		const PlanarOffset& obstacle_offset_1,
		const PlanarOffset& pos_offset,
		double t,
		unsigned int N,
		double length = XGoal)
{
	TestDynamics dyn;
	RosInterfaceTest rostestinterface;
	setupTestCase(rostestinterface, dyn, pos_offset, N, length);


	grid_map::GridMap map({"elevation"});
	map.setFrameId("base_link");
	map["elevation"].setZero();
	grid_map::Position start_pos(8.0, 0.0);
	map.setGeometry(grid_map::Length(20.0, 10.0), 0.5, start_pos);
	for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it)
	{
		grid_map::Position position;
		map.getPosition(*it, position);
		map.at("elevation", *it) = 0.0;
	}
	grid_map::Position pos_obstacle(
			obstacle_offset_0.x,
			obstacle_offset_0.y);
	grid_map::Index index_obstacle;
	map.getIndex(pos_obstacle, index_obstacle);
	grid_map::Position rel_obstacle_pose;
	map.getPosition(index_obstacle, rel_obstacle_pose);
	map.atPosition("elevation", rel_obstacle_pose) = 1.0;
	grid_map_msgs::GridMap message;
	// Obstacle dynamics
	double dvx = (obstacle_offset_1.x - obstacle_offset_0.x)/(t*N);
	double dvy = (obstacle_offset_1.y - obstacle_offset_0.y)/(t*N);
	// Update some messages
	ros::Rate r(hzrate);
	for (unsigned int i = 0; i < 100; i++)
	{
		ros::Time time = ros::Time::now();
		dyn.transformStamped.header.stamp = time;
		dyn.pose.header.stamp = time;
		dyn.velocity.header.stamp = time;
		dyn.l.header.stamp = time;
		grid_map::GridMapRosConverter::toMessage(map, message);
		rostestinterface.publisher_pose.publish(dyn.pose);
		rostestinterface.publisher_velocity.publish(dyn.velocity);
		rostestinterface.publisher_base_waypoints.publish(dyn.l);
		rostestinterface.br.sendTransform(dyn.transformStamped);
		rostestinterface.publisher_grid_map.publish(message);
		//
		r.sleep();
	}
}



#endif /* TEST_TEST_COMMON_HPP_ */
