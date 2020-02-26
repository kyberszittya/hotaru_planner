/*
 * test_basic_operation.cpp
 *
 *  Created on: Feb 23, 2020
 *      Author: kyberszittya
 */


#include "test_common.hpp"

// SKULL ERROR DISMISSED: this would not work any other way




TEST(HotaruLocalPlannerTest, BasicLocalPlannerTest)
{
	PlanarOffset planar_offset;
	planar_offset.x = 10.0;
	planar_offset.y = 0.0;
	PlanarOffset pos_offset;
	pos_offset.x = 0.0;
	pos_offset.y = 0.0;
	testPlanningScenario(planar_offset, pos_offset, 20);
}

TEST(HotaruLocalPlannerTest, BasicLocalPlannerTestObstacleNotClose)
{
	PlanarOffset planar_offset;
	planar_offset.x = 10.0;
	planar_offset.y = -3.0;
	PlanarOffset pos_offset;
	pos_offset.x = 0.0;
	pos_offset.y = 0.0;
	testPlanningScenario(planar_offset, pos_offset, 40);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "local_planner_tester");
  nh.reset(new ros::NodeHandle);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

