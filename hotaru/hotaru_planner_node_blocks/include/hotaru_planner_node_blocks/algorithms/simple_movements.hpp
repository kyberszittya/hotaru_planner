/*
 * simple_movements.hpp
 *
 *  Created on: Sep 9, 2020
 *      Author: kyberszittya
 */


#include <hotaru_planner_node_blocks/blocks/hotaru_planner_node_blocks.hpp>


namespace hotaru
{

namespace planner
{

class ForwardMovementAlgorithm: public blocks::PlannerAlgorithm
{
protected:
	double distance; // Distance in [m]
	unsigned int steps;
public:
	ForwardMovementAlgorithm(unsigned int steps = 100);

	inline void setDistance(const double distance)
	{
		this->distance = distance;
	}

	virtual void calculateTrajectory() override;

};

}

}

