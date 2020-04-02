/*
 * trajectory_slicer.hpp
 *
 *  Created on: Apr 1, 2020
 *      Author: kyberszittya
 */

#ifndef INCLUDE_HOTARU_NODE_ELEMENTS_TRAJECTORY_SLICER_HPP_
#define INCLUDE_HOTARU_NODE_ELEMENTS_TRAJECTORY_SLICER_HPP_

#include <hotaru_msgs/Waypoint.h>
#include <hotaru_msgs/RefinedTrajectory.h>

#include <rei_common/geometric_utilities.hpp>

#include <vector>
#include <iterator>

constexpr double DISTANCE_THRESHOLD = 1e-4;

class TrajectorySlicer
{
private:
protected:

public:

	/**
	 * @brief: Slice trajectory into two parts, rewrites output waypoint lists
	 * @param: ref_trajectory IN
	 * @param:
	 * @returns: whether slicing has been done or not
	 */
	bool sliceTrajectory(
		const hotaru_msgs::RefinedTrajectory& ref_trajectory,
		std::vector<hotaru_msgs::Waypoint>& waypoints_replan,
		std::vector<hotaru_msgs::Waypoint>& waypoints_original,
		const unsigned int wp_idx);

	/**
	 * @brief: sparse trajectory by relative distance threshold
	 */
	void sparseTrajectory(
		const std::vector<hotaru_msgs::Waypoint>& waypoints_input,
		std::vector<hotaru_msgs::Waypoint>& waypoints_res,
		const double distance_threshold);

	/**
	 * @brief: sparse trajectory by percentage
	 */
	void sparseTrajectoryPercentage(
		const std::vector<hotaru_msgs::Waypoint>& waypoints_input,
		std::vector<hotaru_msgs::Waypoint>& waypoints_res,
		const unsigned int percentage);
};

class TrajectoryMerger
{
private:
protected:
public:
	/**
	 * @brief: merge 2 trajectory segments into an outgoing refine trajectory
	 */
	void merge2Trajectories(const std::vector<hotaru_msgs::Waypoint>& waypoints_forward,
			const std::vector<hotaru_msgs::Waypoint>& waypoints_backward,
			hotaru_msgs::RefinedTrajectory& trajectory
			)
	{
		trajectory.waypoints.clear();
		// Copy backward segment
		std::copy(waypoints_backward.begin(), waypoints_backward.end(), std::back_inserter(trajectory.waypoints));
		// Copy forward segment
		std::copy(waypoints_forward.begin(), waypoints_forward.end(), std::back_inserter(trajectory.waypoints));
	}
};

#endif /* INCLUDE_HOTARU_NODE_ELEMENTS_TRAJECTORY_SLICER_HPP_ */
