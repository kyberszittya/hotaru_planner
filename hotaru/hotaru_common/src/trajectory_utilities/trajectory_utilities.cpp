/*
 * trajectory_utilities.cpp
 *
 *  Created on: Feb 26, 2020
 *      Author: kyberszittya
 */

#include <hotaru_common/trajectory_utilities/trajectory_utilities.hpp>


namespace hotaru
{

TrajectorySlicer::TrajectorySlicer(double start_section_ratio):
		lookahead_distance(0.0),
		lookahead_index(0.0),
		section_ratio(start_section_ratio){}

unsigned int TrajectorySlicer::getLookaheadIndex() const
{
	return lookahead_index;
}

double TrajectorySlicer::calcLookaheadDistance(const geometry_msgs::TwistStamped& msg_velocity)
{
	double speed = std::sqrt(msg_velocity.twist.linear.x*msg_velocity.twist.linear.x+
			msg_velocity.twist.linear.y*msg_velocity.twist.linear.y +
			msg_velocity.twist.linear.z*msg_velocity.twist.linear.z);
	lookahead_distance = speed*section_ratio;
	return lookahead_distance;
}

void TrajectorySlicer::calcLookaheadIndex(const autoware_msgs::Lane& lane)
{
	lookahead_index = 0;
	if (lane.waypoints.size() > 2)
	{
		double distance = 0.0;
		for (lookahead_index = 1; lookahead_index < lane.waypoints.size(); lookahead_index++)
		{
			distance += rei::spatialDistance(lane.waypoints[lookahead_index-1].pose.pose.position,
					lane.waypoints[lookahead_index].pose.pose.position);
			if (distance >= lookahead_distance)
			{
				break;
			}
		}
	}
}

void TrajectorySlicer::sliceFromStartToLookahead(
	const autoware_msgs::Lane& lane,
	std::vector<geometry_msgs::PoseStamped>& starting_plan)
{
	starting_plan.clear();
	for (unsigned int i = 0; i < lookahead_index; i++)
	{
		starting_plan.push_back(lane.waypoints[i].pose);
	}
}

void TrajectorySlicer::joinWaypointsWithLocalPlan(
	const autoware_msgs::Lane& lane,
	const std::vector<geometry_msgs::Pose>& starting_plan,
	const geometry_msgs::TwistStamped& current_velocity,
	autoware_msgs::Lane& output_lane)
{
	output_lane.header.stamp = lane.header.stamp;
	for (const auto& v: starting_plan)
	{
		autoware_msgs::Waypoint w;
		w.pose.pose = v;
		w.twist.twist.linear.x = current_velocity.twist.linear.x;
		output_lane.waypoints.push_back(std::move(w));
	}
	for (int i = lookahead_index; i < lane.waypoints.size(); i++)
	{
		output_lane.waypoints.push_back(lane.waypoints[i]);
	}
}
}
