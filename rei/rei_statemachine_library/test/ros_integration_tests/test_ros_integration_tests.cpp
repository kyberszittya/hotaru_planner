/*
 * test_ros_integration_tests.cpp
 *
 *  Created on: Feb 21, 2020
 *      Author: kyberszittya
 */
#include <ros/ros.h>
#include <gtest/gtest.h>

#include <std_msgs/Float64.h>

// SKULL ERROR DISMISSED: this would not work any other way
std::shared_ptr<ros::NodeHandle> nh;

TEST(SyncStateIntegrationTest, ProvideContinousStateUpdate)
{
	ros::Publisher pub_speed = nh->advertise<std_msgs::Float64>("/random_vehicle_speed", 10);
	ros::Publisher pub_vehicle_cmd = nh->advertise<std_msgs::Float64>("/random_vehicle_cmd", 10);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_service_client");
  nh.reset(new ros::NodeHandle);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

