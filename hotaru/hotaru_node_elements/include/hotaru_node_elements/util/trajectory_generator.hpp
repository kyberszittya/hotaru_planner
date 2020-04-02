/*
 * trajectory_generator.hpp
 *
 *  Created on: Apr 1, 2020
 *      Author: kyberszittya
 */

#ifndef INCLUDE_HOTARU_NODE_ELEMENTS_UTIL_TRAJECTORY_GENERATOR_HPP_
#define INCLUDE_HOTARU_NODE_ELEMENTS_UTIL_TRAJECTORY_GENERATOR_HPP_

#include <hotaru_msgs/RefinedTrajectory.h>
#include <hotaru_msgs/Waypoint.h>

/**
 * Factory class for testing purposes
 */
class TestTrajectoryFactory
{
private:
	static TestTrajectoryFactory* instance;
protected:

	TestTrajectoryFactory()
	{
		static bool static_init = []()->bool {
			instance = new TestTrajectoryFactory();
			return true;
		}();
	}
public:

	TestTrajectoryFactory* operator->(){ return instance; }
	const TestTrajectoryFactory* operator->() const { return instance; }
	TestTrajectoryFactory& operator*() { return * instance;}
	const TestTrajectoryFactory& operator*() const { return *instance; }

	void generateTrajectory(hotaru_msgs::RefinedTrajectory& out_trajectory, const unsigned int N, const double ds)
	{
		out_trajectory.waypoints.reserve(N);
		for (unsigned int i = 0; i < N; i++)
		{
			hotaru_msgs::Waypoint wp;
			wp.pose.position.x = i * ds;
			out_trajectory.waypoints.emplace_back(wp);
		}
	}

	void generateWaypointVector(std::vector<hotaru_msgs::Waypoint>& out_trajectory, const unsigned int N, const double ds, double offset_x = 0.0)
	{
		out_trajectory.reserve(N);
		for (unsigned int i = 0; i < N; i++)
		{
			hotaru_msgs::Waypoint wp;
			wp.pose.position.x = i * ds + offset_x;
			out_trajectory.emplace_back(wp);
		}
	}

};

#endif /* INCLUDE_HOTARU_NODE_ELEMENTS_UTIL_TRAJECTORY_GENERATOR_HPP_ */
