/*
 * hotaru_planner_node_blocks.hpp
 *
 *  Created on: Sep 9, 2020
 *      Author: kyberszittya
 */

#ifndef INCLUDE_HOTARU_PLANNER_NODE_BLOCKS_BLOCKS_HOTARU_PLANNER_NODE_BLOCKS_HPP_
#define INCLUDE_HOTARU_PLANNER_NODE_BLOCKS_BLOCKS_HOTARU_PLANNER_NODE_BLOCKS_HPP_

#include <memory>

namespace hotaru
{

namespace blocks
{

class EventReceptor
{

};

class PlannerAlgorithm
{

};

class TrajectorySlicer
{

};

class TrajectoryMerger
{

};

} // namespace blocks

namespace planner
{

class PlannerNode
{
protected:
	std::shared_ptr<blocks::EventReceptor> eventreceptor;
	std::shared_ptr<blocks::PlannerAlgorithm> planneralgorithm;
	std::shared_ptr<blocks::TrajectorySlicer> trajectoryslicer;
	std::shared_ptr<blocks::TrajectoryMerger> trajectorymerger;
public:
};

} // namespace planner

} // namespace hotaru


#endif /* INCLUDE_HOTARU_PLANNER_NODE_BLOCKS_BLOCKS_HOTARU_PLANNER_NODE_BLOCKS_HPP_ */
