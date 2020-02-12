/*
 * common_building_blocks.hpp
 *
 *  Created on: Jan 30, 2020
 *      Author: kyberszittya
 */

#ifndef INCLUDE_HOTARU_COMMON_COMMON_BUILDING_BLOCKS_HPP_
#define INCLUDE_HOTARU_COMMON_COMMON_BUILDING_BLOCKS_HPP_

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <grid_map_ros/grid_map_ros.hpp>

#include <tf/transform_datatypes.h>

#include <Eigen/Dense>

#include <queue>

namespace hotaru
{

struct PlannerKinematicState
{
	geometry_msgs::PoseStamped goal;
	tf::Stamped<tf::Pose> tf_goal;
	geometry_msgs::PoseStamped pose;
	tf::Stamped<tf::Pose> tf_pose;
	Eigen::Vector2d tf_proj_pose;
	geometry_msgs::TwistStamped twist;
};

struct PlannerPerceptionState
{
	grid_map::GridMap global_map;
};

}

#endif /* INCLUDE_HOTARU_COMMON_COMMON_BUILDING_BLOCKS_HPP_ */
