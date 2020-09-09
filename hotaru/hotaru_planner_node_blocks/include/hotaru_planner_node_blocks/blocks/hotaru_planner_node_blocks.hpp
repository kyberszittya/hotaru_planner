/*
 * hotaru_planner_node_blocks.hpp
 *
 *  Created on: Sep 9, 2020
 *      Author: kyberszittya
 */

#ifndef INCLUDE_HOTARU_PLANNER_NODE_BLOCKS_BLOCKS_HOTARU_PLANNER_NODE_BLOCKS_HPP_
#define INCLUDE_HOTARU_PLANNER_NODE_BLOCKS_BLOCKS_HOTARU_PLANNER_NODE_BLOCKS_HPP_

#include <memory>

#include <hotaru_planner_msgs/TrajectoryArray.h>

namespace hotaru
{

namespace blocks
{

class EventReceptor
{

};

class PlannerAlgorithm
{
protected:
	hotaru_planner_msgs::TrajectoryArray resultant_trajectory;
public:
	virtual ~PlannerAlgorithm() {}
	virtual void calculateTrajectory() = 0;

	hotaru_planner_msgs::TrajectoryArray getResultantTrajectory()
	{
		return resultant_trajectory;
	}
};

class TrajectorySlicer
{

};

class TrajectoryMerger
{

};

} // namespace blocks



} // namespace hotaru


#endif /* INCLUDE_HOTARU_PLANNER_NODE_BLOCKS_BLOCKS_HOTARU_PLANNER_NODE_BLOCKS_HPP_ */
