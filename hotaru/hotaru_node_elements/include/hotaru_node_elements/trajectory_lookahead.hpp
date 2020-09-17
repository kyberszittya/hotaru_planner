/*
 * trajectory_lookahead.hpp
 *
 *  Created on: Apr 22, 2020
 *      Author: kyberszittya
 */

#ifndef INCLUDE_HOTARU_NODE_ELEMENTS_TRAJECTORY_LOOKAHEAD_HPP_
#define INCLUDE_HOTARU_NODE_ELEMENTS_TRAJECTORY_LOOKAHEAD_HPP_

#include <rei_common/geometric_utilities.hpp>

namespace hotaru
{

class TrajectoryLookaheadPoint
{
public:
	static int lookaheadPoint(const double constant_lookahead, const geometry_msgs::Twist& vel)
	{
		return static_cast<int>(rei::velocityScalar(vel)*constant_lookahead);
	}
};

}

#endif /* INCLUDE_HOTARU_NODE_ELEMENTS_TRAJECTORY_LOOKAHEAD_HPP_ */
