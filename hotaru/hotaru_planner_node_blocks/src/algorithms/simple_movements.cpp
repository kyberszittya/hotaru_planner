/*
 * simple_movements.cpp
 *
 *  Created on: Sep 9, 2020
 *      Author: kyberszittya
 */

#include <hotaru_planner_node_blocks/algorithms/simple_movements.hpp>

namespace hotaru
{

namespace planner
{

ForwardMovementAlgorithm::ForwardMovementAlgorithm(unsigned int steps): steps(steps), distance(0.0)
	{}



void ForwardMovementAlgorithm::calculateTrajectory()
{
	hotaru_planner_msgs::Trajectory trajectory;
	double x = 0.0;
	double increment = distance/steps;
	if (distance > 0.0)
	{
		hotaru_planner_msgs::Waypoint wp;
		wp.pose.pose.position.x = x;
		wp.pose.pose.position.y = 0.0;
		wp.pose.pose.position.z = 0.0;
		// Orientation forward
		wp.pose.pose.orientation.x = 0.0;
		wp.pose.pose.orientation.y = 0.0;
		wp.pose.pose.orientation.z = 0.0;
		wp.pose.pose.orientation.w = 1.0;
		x += increment;

	}
}


} // namespace planner

} // namespace hotaru


