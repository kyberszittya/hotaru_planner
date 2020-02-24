/*
 * test_ros_integration_timeout.cpp
 *
 *  Created on: Feb 22, 2020
 *      Author: kyberszittya
 */

#include "ros_integration_test_commmon.hpp"

std::shared_ptr<ros::NodeHandle> nh;






TEST(SyncStateIntegrationTest, TimeoutStateUpdate)
{
	ros::Publisher pub_speed = nh->advertise<geometry_msgs::PoseStamped>("/random_vehicle_command", 1);
	StateMachineStateReader sread(nh);
	sread.init();
	geometry_msgs::PoseStamped msg_pose;
	ros::Rate r(100);
	for (int i = 0; i < 40; i++)
	{
		msg_pose.header.stamp = ros::Time::now();
		pub_speed.publish(msg_pose);
		ros::spinOnce();
		r.sleep();
	}
	ASSERT_EQ(sread.getSignal().sig_id, 0x01);
	ros::Rate r_timeout(1);
	r_timeout.sleep();
	ASSERT_EQ(sread.getSignal().sig_id, 0x02);
	for (int i = 0; i < 40; i++)
	{
		msg_pose.header.stamp = ros::Time::now();
		pub_speed.publish(msg_pose);
		ros::spinOnce();
		r.sleep();
	}
	ASSERT_EQ(sread.getSignal().sig_id, 0x01);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_service_client");
  nh.reset(new ros::NodeHandle);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
