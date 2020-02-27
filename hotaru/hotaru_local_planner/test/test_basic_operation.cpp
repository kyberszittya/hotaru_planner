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
	testPlanningScenario(planar_offset, pos_offset, 200, 2);
}


TEST(HotaruLocalPlannerTest, BasicLocalPlannerTestObstacleNotClose)
{
	PlanarOffset planar_offset;
	planar_offset.x = 10.0;
	planar_offset.y = -3.0;
	PlanarOffset pos_offset;
	pos_offset.x = 0.0;
	pos_offset.y = 0.0;
	testPlanningScenario(planar_offset, pos_offset, 200, 2);
}

TEST(HotaruLocalPlannerTest, BasicLocalPlannerTestObstacleLong)
{
	PlanarOffset planar_offset;
	planar_offset.x = 10.0;
	planar_offset.y = 0.0;
	PlanarOffset pos_offset;
	pos_offset.x = 0.0;
	pos_offset.y = 0.0;
	testPlanningScenario(planar_offset, pos_offset, 200, 2, 100);
}



/*
TEST(HotaruLocalPlannerTest, BasicLocalPlannerContinousObstacle)
{
	PlanarOffset obs_offset_0;
	obs_offset_0.x = 10.0;
	obs_offset_0.y = 0.0;
	PlanarOffset obs_offset_1;
	obs_offset_1.x = 10.0;
	obs_offset_1.y = -3.0;
	PlanarOffset pos_offset;
	pos_offset.x = 0.0;
	pos_offset.y = 0.0;
	testPlanningScenarioLinearMovingObstacle(obs_offset_0, obs_offset_1, pos_offset, 4.0, 40);
}
*/

int main(int argc, char **argv)
{
  ros::init(argc, argv, "local_planner_tester");
  nh.reset(new ros::NodeHandle);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

