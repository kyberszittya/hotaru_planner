/*
 * trajectory_slicer.cpp
 *
 *  Created on: Apr 1, 2020
 *      Author: kyberszittya
 */

#include <hotaru_node_elements/trajectory_slicer.hpp>

#include <iostream>

bool TrajectorySlicer::sliceTrajectory(
		const hotaru_msgs::RefinedTrajectory& ref_trajectory,
		std::vector<hotaru_msgs::Waypoint>& waypoints_replan,
		std::vector<hotaru_msgs::Waypoint>& waypoints_original,
		const unsigned int wp_idx)
{
	// If empty or the index bigger than reference trajectory index, we return with failure
	if (ref_trajectory.waypoints.size() == 0 || wp_idx >= ref_trajectory.waypoints.size())
	{
		return false;
	}
	waypoints_original.clear();
	waypoints_replan.clear();
	waypoints_replan.reserve(wp_idx);
	if (wp_idx != 0)
	{
		// Slicing at waypoint index
		std::copy(ref_trajectory.waypoints.begin(),
				ref_trajectory.waypoints.begin() + wp_idx,
				std::back_inserter(waypoints_replan)
		);
		std::copy(ref_trajectory.waypoints.begin() + wp_idx,
				ref_trajectory.waypoints.end(),
				std::back_inserter(waypoints_original)
		);
	}
	else
	{
		waypoints_original.reserve(ref_trajectory.waypoints.size() - wp_idx);
		// No slicing to be done: waypoint index not specified
		std::copy(ref_trajectory.waypoints.begin(),
				ref_trajectory.waypoints.end(),
				std::back_inserter(waypoints_original)
		);
		// We have nothing to do here here from this point on
		// but indicate no slicing done
		return false;
	}
	return true;
}


void TrajectorySlicer::sparseTrajectory(
	const std::vector<hotaru_msgs::Waypoint>& waypoints_input,
	std::vector<hotaru_msgs::Waypoint>& waypoints_res,
	const double distance_threshold)
{
	if (distance_threshold < DISTANCE_THRESHOLD || waypoints_input.size() == 0)
	{
		return;
	}
	double d = 0.0;
	using namespace hotaru_msgs;
	waypoints_res.clear();
	Waypoint wp0 = waypoints_input.front();
	waypoints_res.push_back(wp0);
	// Add waypoints to resulting vector
	int i = 0;
	for (const auto& wp: waypoints_input)
	{
		d += rei::spatialDistance(wp0.pose.pose.position, wp.pose.pose.position);
		if (d > distance_threshold-DISTANCE_THRESHOLD)
		{
			d = 0.0;
			waypoints_res.push_back(wp);
		}
		wp0 = wp;
		i++;
	}
}

void TrajectorySlicer::sparseTrajectoryPercentage(
	const std::vector<hotaru_msgs::Waypoint>& waypoints_input,
	std::vector<hotaru_msgs::Waypoint>& waypoints_res,
	const unsigned int percentage)
{
	if (percentage > 100 || waypoints_input.size() == 0)
	{
		return;
	}
	using namespace hotaru_msgs;
	waypoints_res.clear();
	// Add waypoints to resulting vector
	int di = 1.0/((double)percentage)*waypoints_input.size();
	for (unsigned int i = 0; i < waypoints_input.size(); i+= di)
	{
		waypoints_res.push_back(waypoints_input[i]);
	}
}
