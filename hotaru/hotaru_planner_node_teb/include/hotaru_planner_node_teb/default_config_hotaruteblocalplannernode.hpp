#ifndef INCLUDE_HOTARU_HOTARUTEBLOCALPLANNERNODE_HPP_
#define INCLUDE_HOTARU_HOTARUTEBLOCALPLANNERNODE_HPP_

namespace hotaru
{
	constexpr bool DEFAULT_DEBUG = true
	;
	constexpr bool DEFAULT_BYPASS_BEHAVIOR = false
	;
	namespace planner_setup
	{
		constexpr double DEFAULT_VIA_POINTS_SPEED_LINEAR_RATIO = 1.5
		;
	}
	namespace robot
	{
		constexpr double DEFAULT_WHEELBASE = 2.7
		;
		constexpr double DEFAULT_MIN_TURNING_RADIUS = 10.4
		;
		constexpr double DEFAULT_MAX_VEL_X = 10.0
		;
		constexpr double DEFAULT_MAX_VEL_Y = 0.0
		;
		constexpr double DEFAULT_ACC_LIM_THETA = 0.05
		;
		constexpr double DEFAULT_ACC_LIM_X = 0.2
		;
		constexpr double DEFAULT_MAX_VEL_X_BACKWARDS = 0.01
		;
		constexpr bool DEFAULT_CMD_ANGLE_INSTEAD_ROTVEL = true
		;
	}
	namespace teb_setup
	{
		constexpr bool DEFAULT_HOMOTOPY_ENABLED = false
		;
	}
	namespace teb_trajectory
	{
		constexpr double DEFAULT_DT_REF = 1.0
		;
		constexpr double DEFAULT_DT_HYSTERESIS = 0.1
		;
		constexpr double DEFAULT_MIN_OBSTACLE_DISTANCE = 0.5
		;
		constexpr double DEFAULT_INFLATION_DISTANCE = 0.6
		;
		constexpr double DEFAULT_XY_GOAL_TOLERANCE = 0.4
		;
		constexpr double DEFAULT_YAW_GOAL_TOLERANCE = 0.2
		;
	}
	namespace teb_weights
	{
		constexpr int DEFAULT_NO_INNER_ITERATIONS = 4
		;
		constexpr int DEFAULT_NO_OUTER_ITERATIONS = 5
		;
		constexpr double DEFAULT_PENALTY_EPSILON = 0.1
		;
		constexpr double DEFAULT_WEIGHT_OBSTACLE = 40.0
		;
		constexpr double DEFAULT_WEIGHT_INFLATION = 20.0
		;
		constexpr double DEFAULT_WEIGHT_VIAPOINTS = 4.0
		;
		constexpr double DEFAULT_WEIGHT_OPTIMALTIME = 1.0
		;
		constexpr double DEFAULT_WEIGHT_SHORTEST_PATH = 0.0
		;
		constexpr double DEFAULT_WEIGHT_ACC_LIM_X = 0.0
		;
		constexpr double DEFAULT_WEIGHT_ACC_LIM_Y = 0.0
		;
		constexpr double DEFAULT_WEIGHT_ACC_LIM_THETA = 0.0
		;
		constexpr double DEFAULT_WEIGHT_MAX_VEL_X = 0.0
		;
		constexpr double DEFAULT_WEIGHT_MAX_VEL_Y = 0.0
		;
		constexpr double DEFAULT_WEIGHT_MAX_VEL_THETA = 0.0
		;
		constexpr double DEFAULT_WEIGHT_KINEMATICS_TURNING_RADIUS = 10.0
		;
		constexpr double DEFAULT_WEIGHT_KINEMATICS_FORWARD_DRIVE = 5.0
		;
		constexpr double DEFAULT_WEIGHT_KINEMATICS_NH = 1000.0
		;
		constexpr double DEFAULT_WEIGHT_ADAPT_FACTOR = 2.0
		;
		constexpr double DEFAULT_OBSTACLE_COST_EXPONENT = 1.0
		;
	}
}

#endif

