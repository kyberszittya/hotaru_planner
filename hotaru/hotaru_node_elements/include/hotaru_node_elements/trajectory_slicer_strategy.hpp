/*
 * trajectory_slicer_strategy.hpp
 *
 *  Created on: Apr 12, 2020
 *      Author: kyberszittya
 */

#ifndef INCLUDE_HOTARU_NODE_ELEMENTS_TRAJECTORY_SLICER_STRATEGY_HPP_
#define INCLUDE_HOTARU_NODE_ELEMENTS_TRAJECTORY_SLICER_STRATEGY_HPP_

#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/TransformStamped.h>

#include <hotaru_msgs/RefinedTrajectory.h>
#include <rei_common/geometric_utilities.hpp>

namespace hotaru
{

class TrajectorySlicerStrategy
{
protected:
	int closest_waypoint;
	//
	int slice_point;
public:
	TrajectorySlicerStrategy(): closest_waypoint(-1), slice_point(-1){}

	virtual ~TrajectorySlicerStrategy() = 0;
	void setClosestWaypointIndex(const int closest_waypoint) { this->closest_waypoint = closest_waypoint; }
	int getSlicePoint() { return slice_point; }

	virtual void calcSlicePointIndex(hotaru_msgs::RefinedTrajectory& msg, geometry_msgs::TransformStamped pos_Wro) = 0;
};

class SpeedLookaheadSlicer: public TrajectorySlicerStrategy
{
protected:
	double speed;
	const double speed_ratio;					///< Linear speed-ratio
	const double minimum_lookahead_distance;    ///< Minimum-lookahead distance to be always acheived
public:
	SpeedLookaheadSlicer(const double speed_ratio, const double minimum_lookahead_distance):
		TrajectorySlicerStrategy(),
		speed(0.0),
		speed_ratio(speed_ratio),
		minimum_lookahead_distance(minimum_lookahead_distance){}

	void setSpeed(const double speed) { this->speed = speed; }

	/**
	 * @brief: calculate lookeahead point based on a speed-based policy
	 * @precondition: received closest point > -1 && trajectory.size > 0 -> -1
	 * @precondition: trajectory.size == 1 -> 0
	 */
	virtual void calcSlicePointIndex(hotaru_msgs::RefinedTrajectory& msg, geometry_msgs::TransformStamped pos_Wro)
	{
		if (closest_waypoint < 0 || msg.waypoints.size()==0)
		{
			slice_point = -1;
		}
		else if (msg.waypoints.size()==1)
		{
			slice_point = 0;
		}
		else
		{
			double d = 0.0;
			geometry_msgs::Pose wp_tm1;
			tf2::doTransform(msg.waypoints[closest_waypoint].pose.pose, wp_tm1, pos_Wro);
			geometry_msgs::Pose wp_t;
			for (int i = closest_waypoint+1; i < msg.waypoints.size(); i++)
			{
				tf2::doTransform(msg.waypoints[i].pose.pose, wp_t, pos_Wro);
				d += rei::planarDistance(wp_t.position, wp_tm1.position);
				if (d > minimum_lookahead_distance)
				{
					if (speed_ratio*speed < d)
					{
						slice_point = i;
						break;
					}
				}
				wp_tm1 = wp_t;
			}
		}
	}

};

}

#endif /* INCLUDE_HOTARU_NODE_ELEMENTS_TRAJECTORY_SLICER_STRATEGY_HPP_ */
