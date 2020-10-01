/*
 * hotaru_planner_node.hpp
 *
 *  Created on: Sep 9, 2020
 *      Author: kyberszittya
 */

#ifndef INCLUDE_HOTARU_PLANNER_NODE_BLOCKS_BLOCKS_HOTARU_PLANNER_NODE_HPP_
#define INCLUDE_HOTARU_PLANNER_NODE_BLOCKS_BLOCKS_HOTARU_PLANNER_NODE_HPP_


#include <rei_construct_elements/rei_hybrid_control_node.hpp>
#include <hotaru_planner_node_blocks/blocks/hotaru_planner_node_blocks.hpp>

namespace hotaru
{

namespace planner
{

template<class Subscriber, class Publisher> class PlannerNode
{
protected:
	//
	Subscriber sub_input_trajectory;
	Publisher  pub_refined_trajectory;
	//
	std::unique_ptr<blocks::EventReceptor> eventreceptor;
	std::unique_ptr<blocks::PlannerAlgorithm> planneralgorithm;
	std::unique_ptr<blocks::TrajectorySlicer> trajectoryslicer;
	std::unique_ptr<blocks::TrajectoryMerger> trajectorymerger;
	//
public:
};

} // namespace planner

}


#endif /* INCLUDE_HOTARU_PLANNER_NODE_BLOCKS_BLOCKS_HOTARU_PLANNER_NODE_HPP_ */
