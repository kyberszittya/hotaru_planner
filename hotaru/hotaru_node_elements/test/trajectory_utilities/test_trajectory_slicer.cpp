/*
 * test_trajectory_slicer.cpp
 *
 *  Created on: Apr 1, 2020
 *      Author: kyberszittya
 */


#include <gtest/gtest.h>


#include <hotaru_node_elements/trajectory_slicer.hpp>
#include <hotaru_node_elements/util/trajectory_generator.hpp>

TEST(TrajectorySlicerTest, SimpleTrajectory)
{
	TrajectorySlicer trajectoryslicer;
	hotaru_msgs::RefinedTrajectory ref_trajectory;
	TestTrajectoryFactory* t;
	t->generateTrajectory(ref_trajectory, 100, 1.0);
	using namespace hotaru_msgs;
	std::vector<Waypoint> waypoint_replan;
	std::vector<Waypoint> waypoint_original;
	trajectoryslicer.sliceTrajectory(ref_trajectory, waypoint_replan, waypoint_original,  10);
	ASSERT_EQ(waypoint_replan.size(), 10);
	ASSERT_DOUBLE_EQ(waypoint_replan[0].pose.pose.position.x, 0.0);
	ASSERT_DOUBLE_EQ(waypoint_replan[0].pose.pose.position.y, 0.0);
	ASSERT_DOUBLE_EQ(waypoint_replan[5].pose.pose.position.x, 5.0);
	ASSERT_DOUBLE_EQ(waypoint_replan[5].pose.pose.position.y, 0.0);
	ASSERT_EQ(waypoint_original.size(), 90);
	ASSERT_DOUBLE_EQ(waypoint_original[5].pose.pose.position.x, 15.0);
	ASSERT_DOUBLE_EQ(waypoint_original[5].pose.pose.position.y, 0.0);
	ASSERT_DOUBLE_EQ(waypoint_original[15].pose.pose.position.x, 25.0);
}

TEST(TrajectorySlicerTest, SimpleTrajectoryBiggerWpIdx)
{
	TrajectorySlicer trajectoryslicer;
	hotaru_msgs::RefinedTrajectory ref_trajectory;
	TestTrajectoryFactory* t;
	t->generateTrajectory(ref_trajectory, 100, 1.0);
	using namespace hotaru_msgs;
	std::vector<Waypoint> waypoint_replan;
	std::vector<Waypoint> waypoint_original;
	trajectoryslicer.sliceTrajectory(ref_trajectory, waypoint_replan, waypoint_original,  102);
	ASSERT_EQ(waypoint_replan.size(), 0);
	ASSERT_EQ(waypoint_original.size(), 0);
}


TEST(TrajectorySlicerTest, NoIdxTrajectory)
{
	TrajectorySlicer trajectoryslicer;
	hotaru_msgs::RefinedTrajectory ref_trajectory;
	TestTrajectoryFactory* t;
	t->generateTrajectory(ref_trajectory, 100, 1.0);
	using namespace hotaru_msgs;
	std::vector<Waypoint> waypoint_replan;
	std::vector<Waypoint> waypoint_original;
	trajectoryslicer.sliceTrajectory(ref_trajectory, waypoint_replan, waypoint_original,  0);
	ASSERT_EQ(waypoint_replan.size(), 0);
	ASSERT_EQ(waypoint_original.size(), 100);
}

TEST(TrajectorySlicerTest, NoTrajectory)
{
	TrajectorySlicer trajectoryslicer;
	hotaru_msgs::RefinedTrajectory ref_trajectory;
	using namespace hotaru_msgs;
	std::vector<Waypoint> waypoint_replan;
	std::vector<Waypoint> waypoint_original;
	trajectoryslicer.sliceTrajectory(ref_trajectory, waypoint_replan, waypoint_original,  0);
	ASSERT_EQ(waypoint_replan.size(), 0);
	ASSERT_EQ(waypoint_original.size(), 0);
}

TEST(TrajectorySlicerTest, NoTrajectoryWpIdx)
{
	TrajectorySlicer trajectoryslicer;
	hotaru_msgs::RefinedTrajectory ref_trajectory;
	using namespace hotaru_msgs;
	std::vector<Waypoint> waypoint_replan;
	std::vector<Waypoint> waypoint_original;
	trajectoryslicer.sliceTrajectory(ref_trajectory, waypoint_replan, waypoint_original,  10);
	ASSERT_EQ(waypoint_replan.size(), 0);
	ASSERT_EQ(waypoint_original.size(), 0);
}

TEST(TrajectorySlicerTest, SparseTrajectoryInvalidDistanceThreshold)
{
	TrajectorySlicer trajectoryslicer;
	hotaru_msgs::RefinedTrajectory ref_trajectory;
	TestTrajectoryFactory* t;
	t->generateTrajectory(ref_trajectory, 100, 1.0);
	using namespace hotaru_msgs;
	std::vector<Waypoint> waypoint_res;
	trajectoryslicer.sparseTrajectory(ref_trajectory.waypoints, waypoint_res, -0.1);
	ASSERT_EQ(waypoint_res.size(), 0);
}

TEST(TrajectorySlicerTest, SparseTrajectoryDistanceThreshold)
{
	TrajectorySlicer trajectoryslicer;
	hotaru_msgs::RefinedTrajectory ref_trajectory;
	TestTrajectoryFactory* t;
	t->generateTrajectory(ref_trajectory, 100, 1.0);
	using namespace hotaru_msgs;
	std::vector<Waypoint> waypoint_res;
	trajectoryslicer.sparseTrajectory(ref_trajectory.waypoints, waypoint_res, 5.0);
	ASSERT_EQ(waypoint_res.size(), 20);
}

TEST(TrajectorySlicerTest, SparseTrajectoryDistanceThresholdEmptyTrajectory)
{
	TrajectorySlicer trajectoryslicer;
	hotaru_msgs::RefinedTrajectory ref_trajectory;
	TestTrajectoryFactory* t;
	using namespace hotaru_msgs;
	std::vector<Waypoint> waypoint_res;
	trajectoryslicer.sparseTrajectory(ref_trajectory.waypoints, waypoint_res, 5.0);
	ASSERT_EQ(waypoint_res.size(), 0);
	trajectoryslicer.sparseTrajectory(ref_trajectory.waypoints, waypoint_res, -1.0);
	ASSERT_EQ(waypoint_res.size(), 0);
}

TEST(TrajectorySlicerTest, SparseTrajectoryPercentage)
{
	TrajectorySlicer trajectoryslicer;
	hotaru_msgs::RefinedTrajectory ref_trajectory;
	TestTrajectoryFactory* t;
	t->generateTrajectory(ref_trajectory, 100, 1.0);
	using namespace hotaru_msgs;
	std::vector<Waypoint> waypoint_res;
	trajectoryslicer.sparseTrajectoryPercentage(ref_trajectory.waypoints, waypoint_res, 50);
	ASSERT_EQ(waypoint_res.size(), 50);
}

TEST(TrajectorySlicerTest, SparseTrajectoryInvalidPercentage)
{
	TrajectorySlicer trajectoryslicer;
	hotaru_msgs::RefinedTrajectory ref_trajectory;
	TestTrajectoryFactory* t;
	t->generateTrajectory(ref_trajectory, 100, 1.0);
	using namespace hotaru_msgs;
	std::vector<Waypoint> waypoint_res;
	trajectoryslicer.sparseTrajectoryPercentage(ref_trajectory.waypoints, waypoint_res, 190);
	ASSERT_EQ(waypoint_res.size(), 0);
}

TEST(TrajectorySlicerTest, SparseTrajectoryPercentageEmptyTrajectory)
{
	TrajectorySlicer trajectoryslicer;
	hotaru_msgs::RefinedTrajectory ref_trajectory;
	TestTrajectoryFactory* t;
	using namespace hotaru_msgs;
	std::vector<Waypoint> waypoint_res;
	trajectoryslicer.sparseTrajectoryPercentage(ref_trajectory.waypoints, waypoint_res, 50);
		ASSERT_EQ(waypoint_res.size(), 0);
	trajectoryslicer.sparseTrajectoryPercentage(ref_trajectory.waypoints, waypoint_res, 190);
	ASSERT_EQ(waypoint_res.size(), 0);
}

// Trajectory merger

TEST(TrajectoryMergerTest, SparseTrajectoryPercentageEmptyTrajectory)
{
	TrajectoryMerger trajectory_merger;
	hotaru_msgs::RefinedTrajectory ref_trajectory;
	TestTrajectoryFactory* t;
	using namespace hotaru_msgs;
	std::vector<Waypoint> waypoint_forward;
	std::vector<Waypoint> waypoint_backward;
	t->generateWaypointVector(waypoint_forward, 100, 1.0, 100.0);
	t->generateWaypointVector(waypoint_backward, 100, 1.0);
	RefinedTrajectory traj;
	trajectory_merger.merge2Trajectories(waypoint_forward, waypoint_backward, traj);
	ASSERT_EQ(traj.waypoints.size(), 200);
	ASSERT_DOUBLE_EQ(traj.waypoints[0].pose.pose.position.x, 0.0);
	ASSERT_DOUBLE_EQ(traj.waypoints[100].pose.pose.position.x, 100.0);
	ASSERT_DOUBLE_EQ(traj.waypoints[199].pose.pose.position.x, 199.0);
}

int main(int argc, char** argv)
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
