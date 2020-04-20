/*
 * lib_rei_common.cpp
 *
 *  Created on: Feb 26, 2020
 *      Author: kyberszittya
 */

#include <rei_common/geometric_utilities.hpp>

namespace rei
{



double distanceToLine(const geometry_msgs::Point& p0,
		const geometry_msgs::Point& p1,
		const geometry_msgs::Point& o)
{
	double dxe = p1.x - p0.x;
	double dye = p1.y - p0.y;

	return (dye*o.x - dxe*o.y + p1.x * p0.y - p1.y * p0.x)/
			std::sqrt(dye*dye+dxe*dxe);
}

double distanceToLine(const geometry_msgs::Point& p0,
		const geometry_msgs::Point& p1,
		const geometry_msgs::Point32& o)
{
	double dxe = p1.x - p0.x;
	double dye = p1.y - p0.y;

	return (dye*o.x - dxe*o.y + p1.x * p0.y - p1.y * p0.x)/
			std::sqrt(dye*dye+dxe*dxe);
}

double spatialDistance(const geometry_msgs::Point& msg0,
		const geometry_msgs::Point& msg1)
{
	double dx = msg0.x - msg1.x;
	double dy = msg0.y - msg1.y;
	double dz = msg0.z - msg1.z;
	return std::sqrt(dx*dx+dy*dy+dz*dz);
}

double planarDistance(const geometry_msgs::Point& msg0,
		const geometry_msgs::Point& msg1)
{
	double dx = msg0.x - msg1.x;
	double dy = msg0.y - msg1.y;
	return std::sqrt(dx*dx+dy*dy);
}

}

