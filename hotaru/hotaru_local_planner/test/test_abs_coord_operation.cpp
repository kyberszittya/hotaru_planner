/*
 * test_abs_coord_operation.cpp
 *
 *  Created on: Feb 25, 2020
 *      Author: kyberszittya
 */


#include "test_common.hpp"

// SKULL ERROR DISMISSED: this would not work any other way
//std::shared_ptr<ros::NodeHandle> nh;



TEST(HotaruLocalPlannerTest, BasicLocalPlannerGeoInfTest)
{
	PlanarOffset planar_offset;
	planar_offset.x = 10.0;
	planar_offset.y = 0.0;
	PlanarOffset pos_offset;
	pos_offset.x = 612987.1;
	pos_offset.y = 546120.5;
	testPlanningScenario(planar_offset, pos_offset, 20);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "local_planner_tester");
  nh.reset(new ros::NodeHandle);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
