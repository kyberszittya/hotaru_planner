/*
 * trajectory_utilities.hpp
 *
 *  Created on: Feb 26, 2020
 *      Author: kyberszittya
 */

#ifndef INCLUDE_HOTARU_COMMON_TRAJECTORY_UTILITIES_TRAJECTORY_UTILITIES_HPP_
#define INCLUDE_HOTARU_COMMON_TRAJECTORY_UTILITIES_TRAJECTORY_UTILITIES_HPP_

#include <autoware_msgs/Lane.h>

#include <rei_common/geometric_utilities.hpp>

namespace hotaru
{

class TrajectorySlicer
{
protected:
	double lookahead_distance;
	unsigned int lookahead_index;
	double section_ratio;
public:
	TrajectorySlicer(double start_section_ratio = 2.0);

	double calcLookaheadDistance(const geometry_msgs::TwistStamped& msg_velocity);

	void calcLookaheadIndex(const autoware_msgs::Lane& lane);

	unsigned int getLookaheadIndex() const;

	void sliceFromStartToLookahead(const autoware_msgs::Lane& lane,
			std::vector<geometry_msgs::PoseStamped>& starting_plan);

	void joinWaypointsWithLocalPlan(const autoware_msgs::Lane& lane,
			const std::vector<geometry_msgs::Pose>& starting_plan,
			const geometry_msgs::TwistStamped& current_velocity,
			autoware_msgs::Lane& output_lane);
};

}

#endif /* INCLUDE_HOTARU_COMMON_TRAJECTORY_UTILITIES_TRAJECTORY_UTILITIES_HPP_ */
