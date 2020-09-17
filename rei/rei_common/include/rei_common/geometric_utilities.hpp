/*
 * geometric_utilities.hpp
 *
 *  Created on: Feb 26, 2020
 *      Author: kyberszittya
 */

#ifndef INCLUDE_REI_COMMON_GEOMETRIC_UTILITIES_HPP_
#define INCLUDE_REI_COMMON_GEOMETRIC_UTILITIES_HPP_

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Twist.h>

namespace rei
{

double distanceToLine(const geometry_msgs::Point& p0,
		const geometry_msgs::Point& p1,
		const geometry_msgs::Point& o);
double distanceToLine(const geometry_msgs::Point& p0,
		const geometry_msgs::Point& p1,
		const geometry_msgs::Point32& o);
double spatialDistance(const geometry_msgs::Point& msg0,
		const geometry_msgs::Point& msg1);
double planarDistance(const geometry_msgs::Point& msg0,
		const geometry_msgs::Point& msg1);

inline double velocityScalar(const geometry_msgs::Twist& msg)
{
	return sqrt(msg.linear.x*msg.linear.x + msg.linear.y * msg.linear.y + msg.linear.z * msg.linear.z);
}

inline bool isInvalidPoint(const geometry_msgs::Point p0)
{
	return std::isnan(p0.x) || std::isnan(p0.y) || std::isnan(p0.z);
}

}


#endif /* INCLUDE_REI_COMMON_GEOMETRIC_UTILITIES_HPP_ */
