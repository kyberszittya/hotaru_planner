/*
 * test_velocity_profile.cpp
 *
 *  Created on: Apr 24, 2020
 *      Author: kyberszittya
 */

#include <gtest/gtest.h>


#include <hotaru_node_elements/velocity_profile/trapezoidal_velocity.hpp>

using namespace hotaru;
using namespace hotaru_msgs;

hotaru::TrapezoidalVelocityProfile createVelocityProfile(int start_index = 0, int end_index=100)
{
	hotaru::TrapezoidalVelocityProfile profile(0.2);
	Waypoint wp0;
	wp0.twist.twist.linear.x = 1.0;
	Waypoint wp_mid;
	wp_mid.twist.twist.linear.x = 5.0;
	Waypoint wp_end;
	wp_end.twist.twist.linear.x = 10.0;
	profile.setStartWaypoint(start_index, wp0);
	profile.setMidWaypoint(wp_mid);
	profile.setEndWaypoint(end_index, wp_end);
	profile.calc();
	return profile;
}

TEST(TrajectoryVelocityProfile, AtControlPointsEqualLengthStartPoint)
{
	auto profile = createVelocityProfile();
	auto v = profile.getVelocityAtPoint(0, 100);
	ASSERT_DOUBLE_EQ(1.0, v.twist.linear.x);
}

TEST(TrajectoryVelocityProfile, AtControlPointsEqualLengthMidPoint)
{
	auto profile = createVelocityProfile();
	auto v = profile.getVelocityAtPoint(50, 100);
	ASSERT_DOUBLE_EQ(5.0, v.twist.linear.x);

}

TEST(TrajectoryVelocityProfile, AtControlPointsEqualLengthEndPoint)
{
	auto profile = createVelocityProfile();
	auto v = profile.getVelocityAtPoint(100, 100);
	ASSERT_DOUBLE_EQ(10.0, v.twist.linear.x);
}

TEST(TrajectoryVelocityProfile, AtControlPointsEqualLengthStartMiddle)
{
	auto profile = createVelocityProfile();
	auto v = profile.getVelocityAtPoint(20, 100);
	ASSERT_DOUBLE_EQ(3.0, v.twist.linear.x);
}

TEST(TrajectoryVelocityProfile, AtControlPointsEqualLengthEndMiddle)
{
	auto profile = createVelocityProfile();
	auto v = profile.getVelocityAtPoint(80, 100);
	ASSERT_DOUBLE_EQ(7.5, v.twist.linear.x);
}

TEST(TrajectoryVelocityProfile, AtControlPointsEqualLongerStartMiddle)
{
	auto profile = createVelocityProfile();
	auto v = profile.getVelocityAtPoint(30, 150);
	ASSERT_DOUBLE_EQ(3.0, v.twist.linear.x);
}

TEST(TrajectoryVelocityProfile, AtControlPointsLongerEndMiddle)
{
	auto profile = createVelocityProfile();
	auto v = profile.getVelocityAtPoint(160, 200);
	ASSERT_DOUBLE_EQ(7.5, v.twist.linear.x);
}

TEST(TrajectoryVelocityProfile, AtControlPointsLongerEndMiddleShifted)
{
	auto profile = createVelocityProfile(50, 150);
	auto v = profile.getVelocityAtPoint(160, 200);
	ASSERT_DOUBLE_EQ(7.5, v.twist.linear.x);
}

int main(int argc, char** argv)
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
