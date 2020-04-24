/*
 * trapezoidal_velocity.hpp
 *
 *  Created on: Apr 24, 2020
 *      Author: kyberszittya
 */

#ifndef INCLUDE_HOTARU_NODE_ELEMENTS_VELOCITY_PROFILE_TRAPEZOIDAL_VELOCITY_HPP_
#define INCLUDE_HOTARU_NODE_ELEMENTS_VELOCITY_PROFILE_TRAPEZOIDAL_VELOCITY_HPP_


#include <hotaru_msgs/RefinedTrajectory.h>
#include <geometry_msgs/TwistStamped.h>

namespace hotaru
{

class VelocityProfile
{
protected:
	int start_index;
	hotaru_msgs::Waypoint startpoint;
	int mid_index;
	hotaru_msgs::Waypoint midpoint;
	int end_index;
	hotaru_msgs::Waypoint endpoint;
	// Velocity
	geometry_msgs::TwistStamped vel;
public:
	VelocityProfile():
		start_index(-1),
		mid_index(-1),
		end_index(-1)
	{
		vel.header.frame_id = "base_link";

	}

	void setStartWaypoint(int index, const hotaru_msgs::Waypoint wpoint)
	{
		startpoint = wpoint;
		start_index = index;
	}
	void setMidWaypoint(const hotaru_msgs::Waypoint wpoint)
	{
		midpoint = wpoint;
	}
	void setEndWaypoint(int index, const hotaru_msgs::Waypoint wpoint)
	{
		endpoint = wpoint;
		end_index = index;
	}
	virtual void calc() = 0;
	virtual geometry_msgs::TwistStamped getVelocityAtPoint(int i, unsigned int length) = 0;
};

class TrapezoidalVelocityProfile: public VelocityProfile
{
private:
	double step_ratio;
	int midpoint_length;
	int length;

public:

	TrapezoidalVelocityProfile(const double step_ratio): VelocityProfile(), step_ratio(step_ratio){}

	virtual void calc()
	{
		length = end_index - start_index;
		end_index -= start_index;
		start_index = 0;
		mid_index = static_cast<int>((end_index + start_index)/2);
		midpoint_length = length * step_ratio;
	}

	/**
	 * |                   /end
	 * |           mid  d2/
	 * |  d1______|______/
	 * |   /|            |
	 * |  / | step_length|
	 * | /
	 * |/____________________
	 * start
	 */
	virtual geometry_msgs::TwistStamped getVelocityAtPoint(int i, unsigned int new_traj_length) override
	{
		// Calculate parameter t
		double t = (i - start_index)*(static_cast<double>(length)/new_traj_length);
		//
		int d1 = mid_index - midpoint_length/2;
		int d2 = mid_index + midpoint_length/2;
		vel.twist.linear.x = 0.0;
		if (t >= start_index && t <= end_index && end_index != 0)
		{
			if (t < d1) vel.twist.linear.x = startpoint.twist.twist.linear.x + (midpoint.twist.twist.linear.x - startpoint.twist.twist.linear.x)*(t/(d1 - start_index));
			else if (t >= d1 && t < d2) vel.twist.linear.x = midpoint.twist.twist.linear.x;
			else
			{
				double dv = endpoint.twist.twist.linear.x - midpoint.twist.twist.linear.x;
				vel.twist.linear.x = midpoint.twist.twist.linear.x +
						dv*(t - d2)/(end_index - d2);
			}
		}
		return vel;
	}

};

}


#endif /* INCLUDE_HOTARU_NODE_ELEMENTS_VELOCITY_PROFILE_TRAPEZOIDAL_VELOCITY_HPP_ */
