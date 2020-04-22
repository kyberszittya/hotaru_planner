/*
 * hotaru_autoware_brige_node.cpp
 *
 *  Created on: Apr 2, 2020
 *      Author: kyberszittya
 */

#include <map>

#include <ros/ros.h>

#include <hotaru_msgs/RefinedTrajectory.h>
#include <autoware_msgs/Lane.h>
#include <autoware_msgs/LaneArray.h>

constexpr const char* AUTOWARE_LANE_INPUT_ARRAY = "lane_waypoints_array";

class HotaruAutowareBridge
{
private:
	ros::NodeHandle nh;
	std::map<std::string, bool> received_msg_topics;
	autoware_msgs::Lane l;
	// Republish lane array
	ros::Subscriber sub_traffic_lane_array;
	ros::Subscriber sub_intermediate_trajectory;
	ros::Subscriber sub_final_trajectory;
	ros::Publisher  pub_traffic_lane_array;
	autoware_msgs::Lane base_lane;
	ros::Publisher  pub_base_waypoints;
	autoware_msgs::Lane final_lane;
	ros::Publisher  pub_final_waypoints;
	hotaru_msgs::RefinedTrajectory trajectory;
	ros::Publisher pub_trajectory_following_waypoints;
	// Timer to publish initial reference trajectory periodically and latch it
	ros::Timer timer_pub_trajectory;
	std::string lane_frame;
protected:
public:
	HotaruAutowareBridge(const ros::NodeHandle& nh): nh(nh)
	{}

	void cbTrafficLaneArray(const autoware_msgs::LaneArray::ConstPtr& msg)
	{
		l = msg->lanes[0];
		for (const auto& wp: l.waypoints)
		{
			hotaru_msgs::Waypoint wp0;
			wp0.pose = wp.pose;
			wp0.twist = wp.twist;
			trajectory.waypoints.push_back(wp0);
		}
		if (!received_msg_topics["lane_waypoints_array"])
		{
			trajectory.header.frame_id = l.header.frame_id;
			trajectory.header.seq = l.header.seq;
			trajectory.header.stamp = l.header.stamp;
			ROS_INFO("Publishing periodically lane_waypoints_array as input_trajectory");
			received_msg_topics["traffic_waypoints_array"] |= true;
			timer_pub_trajectory = nh.createTimer(ros::Duration(5.0), &HotaruAutowareBridge::cbTimerTrafficLaneEvent, this);
			timer_pub_trajectory.start();
		}

	}



	void cbTimerTrafficLaneEvent(const ros::TimerEvent& e)
	{
		if (pub_traffic_lane_array.getNumSubscribers() > 0)
		{
			pub_traffic_lane_array.publish(trajectory);
		}
	}

	static void convertHotaruToAutowareTrajectory(const hotaru_msgs::RefinedTrajectory::ConstPtr& msg, autoware_msgs::Lane& lane)
	{
		lane.header = msg->header;
		lane.waypoints.clear();
		for (const auto& m: msg->waypoints)
		{
			autoware_msgs::Waypoint wp;
			wp.pose = m.pose;
			wp.twist = m.twist;
			lane.waypoints.emplace_back(wp);
		}
	}

	void cbIntermediateTrajectory(const hotaru_msgs::RefinedTrajectory::ConstPtr& msg)
	{
		//convertHotaruToAutowareTrajectory(msg, base_lane);
		//pub_base_waypoints.publish(base_lane);
	}

	void cbFinalTrajectory(const hotaru_msgs::RefinedTrajectory::ConstPtr& msg)
	{
		convertHotaruToAutowareTrajectory(msg, final_lane);
		pub_final_waypoints.publish(final_lane);
		pub_trajectory_following_waypoints.publish(final_lane);
	}

	bool init()
	{
		sub_traffic_lane_array = nh.subscribe<autoware_msgs::LaneArray>("lane_waypoints_array", 1,
				&HotaruAutowareBridge::cbTrafficLaneArray, this);
		// Setup map
		received_msg_topics["lane_waypoints_array"] = false;
		//
		pub_traffic_lane_array = nh.advertise<hotaru_msgs::RefinedTrajectory>("input_trajectory", 1, true);
		// Sub and pub from hotaru architecture
		//pub_base_waypoints = nh.advertise<autoware_msgs::Lane>("base_waypoints", 1);
		pub_final_waypoints = nh.advertise<autoware_msgs::Lane>("final_waypoints", 1);
		pub_trajectory_following_waypoints = nh.advertise<autoware_msgs::Lane>("mpc_waypoints", 1);
		//sub_intermediate_trajectory = nh.subscribe("refined_trajectory", 1, &HotaruAutowareBridge::cbIntermediateTrajectory, this);
		sub_final_trajectory = nh.subscribe("refined_trajectory", 1, &HotaruAutowareBridge::cbFinalTrajectory, this);
		return true;
	}

	void stop()
	{
		if (timer_pub_trajectory.hasStarted())
		{
			timer_pub_trajectory.stop();
		}
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "hotaru_autoware_bridge_waypoint");
	ros::NodeHandle nh;
	HotaruAutowareBridge bridge(nh);
	if (bridge.init())
	{
		ROS_INFO("Successfully initialized node");
		ros::AsyncSpinner spinner(8);
		spinner.start();
		ros::waitForShutdown();
		bridge.stop();
		spinner.stop();
		return 0;
	}
	ROS_FATAL("Unable to initialize node");
	return -1;
}

