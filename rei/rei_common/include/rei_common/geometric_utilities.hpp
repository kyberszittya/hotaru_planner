/*
 * geometric_utilities.hpp
 *
 *  Created on: Feb 26, 2020
 *      Author: kyberszittya
 */

#ifndef INCLUDE_REI_COMMON_GEOMETRIC_UTILITIES_HPP_
#define INCLUDE_REI_COMMON_GEOMETRIC_UTILITIES_HPP_

#include <geometry_msgs/Point.h>

namespace rei
{

double distanceToLine(const geometry_msgs::Point& p0,
		const geometry_msgs::Point& p1,
		const geometry_msgs::Point& o);
double spatialDistance(const geometry_msgs::Point& msg0,
		const geometry_msgs::Point& msg1);
double planarDistance(const geometry_msgs::Point& msg0,
		const geometry_msgs::Point& msg1);
}


#endif /* INCLUDE_REI_COMMON_GEOMETRIC_UTILITIES_HPP_ */
