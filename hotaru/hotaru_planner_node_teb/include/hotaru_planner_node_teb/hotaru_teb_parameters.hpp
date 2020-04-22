/*
 * hotaru_teb_parameters.hpp
 *
 *  Created on: Apr 21, 2020
 *      Author: kyberszittya
 */

#ifndef INCLUDE_HOTARU_PLANNER_NODE_TEB_HOTARU_TEB_PARAMETERS_HPP_
#define INCLUDE_HOTARU_PLANNER_NODE_TEB_HOTARU_TEB_PARAMETERS_HPP_


namespace hotaru
{

namespace teb
{
// Trajectory parameters
constexpr double DEFAULT_TEB_CONFIG_TRAJECTORY_DT_REF = 1.5;
constexpr double DEFAULT_TEB_CONFIG_TRAJECTORY_DT_HYSTERESIS = 0.6;
//
constexpr double DEFAULT_MIN_TURNING_RADIUS = 10.4;
constexpr double DEFAULT_WHEELBASE = 2.7;
//
constexpr double DEFAULT_MIN_OBSTACLE_DISTANCE = 0.5;
constexpr double DEFAULT_MIN_INFLATION = 0.6;
//
constexpr double DEFAULT_WEIGHT_VIAPOINTS = 15;
constexpr double DEFAULT_WEIGHT_OBSTACLE = 60;
constexpr double DEFAULT_WEIGHT_KINEMATICS_TURNING_RADIUS = 5.0;
constexpr double DEFAULT_WEIGHT_KINEMATICS_NH = 1000.0;
//
constexpr double DEFAULT_XY_GOAL_TOLERANCE = 0.4;
constexpr double DEFAULT_YAW_GOAL_TOLERANCE = 0.2;

}

}


#endif /* INCLUDE_HOTARU_PLANNER_NODE_TEB_HOTARU_TEB_PARAMETERS_HPP_ */
