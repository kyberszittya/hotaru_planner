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
	double min_lookahead_distance;
	double lookahead_distance;
	unsigned int lookahead_index;
	double section_ratio;
	unsigned int offset;
public:
	TrajectorySlicer(double start_section_ratio = 2.0, double min_lookahead_distance = 3.0);

	double calcLookaheadDistance(
			const geometry_msgs::TwistStamped& msg_velocity,
			const double& ref_velocity,
			int velocity_length = 10);

	void calcLookaheadIndex(const autoware_msgs::Lane& lane);

	unsigned int getLookaheadIndex() const;

	void sliceFromStartToLookahead(const autoware_msgs::Lane& lane,
			const geometry_msgs::PoseStamped current_pose,
			std::vector<geometry_msgs::PoseStamped>& starting_plan,
			int skip=0);

	void joinWaypointsWithLocalPlan(const autoware_msgs::Lane& lane,
			const std::vector<geometry_msgs::Pose>& starting_plan,
			const double& current_velocity,
			autoware_msgs::Lane& output_lane);

	void setOffset(int offset)
	{
		if (offset > -1)
		{
			this->offset = offset;
		}
	}
};

}

#endif /* INCLUDE_HOTARU_COMMON_TRAJECTORY_UTILITIES_TRAJECTORY_UTILITIES_HPP_ */
