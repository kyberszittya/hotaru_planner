#include "hotaru_planner_node_teb/hotaruteblocalplannernode.hpp"
#include "hotaru_planner_node_teb/default_config_hotaruteblocalplannernode.hpp"
#include <hotaru_planner_node_teb/planner_setup_struct.hpp>

namespace hotaru
{

void HotaruTebLocalPlannerNode::genParamConfig()
{
	//
	// GROUP planner_setup
	// Load param via_points_speed_linear_ratio
	if (!private_nh->getParam("planner_setup/via_points_speed_linear_ratio", planner_config.via_points_speed_linear_ratio))
	{
		planner_config.via_points_speed_linear_ratio = planner_setup::DEFAULT_VIA_POINTS_SPEED_LINEAR_RATIO;
	}
	ROS_INFO_STREAM("Using planner_setup/via_points_speed_linear_ratio:=" << planner_config.via_points_speed_linear_ratio);
	//
	// GROUP robot
	// Load param wheelbase
	if (!private_nh->getParam("robot/wheelbase", tebconfig.robot.wheelbase))
	{
		tebconfig.robot.wheelbase = robot::DEFAULT_WHEELBASE;
	}
	ROS_INFO_STREAM("Using robot/wheelbase:=" << tebconfig.robot.wheelbase);
	// Load param min_turning_radius
	if (!private_nh->getParam("robot/min_turning_radius", tebconfig.robot.min_turning_radius))
	{
		tebconfig.robot.min_turning_radius = robot::DEFAULT_MIN_TURNING_RADIUS;
	}
	ROS_INFO_STREAM("Using robot/min_turning_radius:=" << tebconfig.robot.min_turning_radius);
	// Load param max_vel_y
	if (!private_nh->getParam("robot/max_vel_y", tebconfig.robot.max_vel_y))
	{
		tebconfig.robot.max_vel_y = robot::DEFAULT_MAX_VEL_Y;
	}
	ROS_INFO_STREAM("Using robot/max_vel_y:=" << tebconfig.robot.max_vel_y);
	// Load param acc_lim_theta
	if (!private_nh->getParam("robot/acc_lim_theta", tebconfig.robot.acc_lim_theta))
	{
		tebconfig.robot.acc_lim_theta = robot::DEFAULT_ACC_LIM_THETA;
	}
	ROS_INFO_STREAM("Using robot/acc_lim_theta:=" << tebconfig.robot.acc_lim_theta);
	// Load param acc_lim_x
	if (!private_nh->getParam("robot/acc_lim_x", tebconfig.robot.acc_lim_x))
	{
		tebconfig.robot.acc_lim_x = robot::DEFAULT_ACC_LIM_X;
	}
	ROS_INFO_STREAM("Using robot/acc_lim_x:=" << tebconfig.robot.acc_lim_x);
	// Load param max_vel_x_backwards
	if (!private_nh->getParam("robot/max_vel_x_backwards", tebconfig.robot.max_vel_x_backwards))
	{
		tebconfig.robot.max_vel_x_backwards = robot::DEFAULT_MAX_VEL_X_BACKWARDS;
	}
	ROS_INFO_STREAM("Using robot/max_vel_x_backwards:=" << tebconfig.robot.max_vel_x_backwards);
	// Load param cmd_angle_instead_rotvel
	if (!private_nh->getParam("robot/cmd_angle_instead_rotvel", tebconfig.robot.cmd_angle_instead_rotvel))
	{
		tebconfig.robot.cmd_angle_instead_rotvel = robot::DEFAULT_CMD_ANGLE_INSTEAD_ROTVEL;
	}
	ROS_INFO_STREAM("Using robot/cmd_angle_instead_rotvel:=" << tebconfig.robot.cmd_angle_instead_rotvel);
	//
	// GROUP teb_setup
	// Load param homotopy_enabled
	if (!private_nh->getParam("teb_setup/homotopy_enabled", tebconfig.hcp.enable_homotopy_class_planning))
	{
		tebconfig.hcp.enable_homotopy_class_planning = teb_setup::DEFAULT_HOMOTOPY_ENABLED;
	}
	ROS_INFO_STREAM("Using teb_setup/homotopy_enabled:=" << tebconfig.hcp.enable_homotopy_class_planning);
	//
	// GROUP teb_trajectory
	// Load param dt_ref
	if (!private_nh->getParam("teb_trajectory/dt_ref", tebconfig.trajectory.dt_ref))
	{
		tebconfig.trajectory.dt_ref = teb_trajectory::DEFAULT_DT_REF;
	}
	ROS_INFO_STREAM("Using teb_trajectory/dt_ref:=" << tebconfig.trajectory.dt_ref);
	// Load param dt_hysteresis
	if (!private_nh->getParam("teb_trajectory/dt_hysteresis", tebconfig.trajectory.dt_hysteresis))
	{
		tebconfig.trajectory.dt_hysteresis = teb_trajectory::DEFAULT_DT_HYSTERESIS;
	}
	ROS_INFO_STREAM("Using teb_trajectory/dt_hysteresis:=" << tebconfig.trajectory.dt_hysteresis);
	// Load param min_obstacle_distance
	if (!private_nh->getParam("teb_trajectory/min_obstacle_distance", tebconfig.obstacles.min_obstacle_dist))
	{
		tebconfig.obstacles.min_obstacle_dist = teb_trajectory::DEFAULT_MIN_OBSTACLE_DISTANCE;
	}
	ROS_INFO_STREAM("Using teb_trajectory/min_obstacle_distance:=" << tebconfig.obstacles.min_obstacle_dist);
	// Load param inflation_distance
	if (!private_nh->getParam("teb_trajectory/inflation_distance", tebconfig.obstacles.inflation_dist))
	{
		tebconfig.obstacles.inflation_dist = teb_trajectory::DEFAULT_INFLATION_DISTANCE;
	}
	ROS_INFO_STREAM("Using teb_trajectory/inflation_distance:=" << tebconfig.obstacles.inflation_dist);
	// Load param xy_goal_tolerance
	if (!private_nh->getParam("teb_trajectory/xy_goal_tolerance", tebconfig.goal_tolerance.xy_goal_tolerance))
	{
		tebconfig.goal_tolerance.xy_goal_tolerance = teb_trajectory::DEFAULT_XY_GOAL_TOLERANCE;
	}
	ROS_INFO_STREAM("Using teb_trajectory/xy_goal_tolerance:=" << tebconfig.goal_tolerance.xy_goal_tolerance);
	// Load param yaw_goal_tolerance
	if (!private_nh->getParam("teb_trajectory/yaw_goal_tolerance", tebconfig.goal_tolerance.yaw_goal_tolerance))
	{
		tebconfig.goal_tolerance.yaw_goal_tolerance = teb_trajectory::DEFAULT_YAW_GOAL_TOLERANCE;
	}
	ROS_INFO_STREAM("Using teb_trajectory/yaw_goal_tolerance:=" << tebconfig.goal_tolerance.yaw_goal_tolerance);
	//
	// GROUP teb_weights
	// Load param weight_obstacle
	if (!private_nh->getParam("teb_weights/weight_obstacle", tebconfig.optim.weight_obstacle))
	{
		tebconfig.optim.weight_obstacle = teb_weights::DEFAULT_WEIGHT_OBSTACLE;
	}
	ROS_INFO_STREAM("Using teb_weights/weight_obstacle:=" << tebconfig.optim.weight_obstacle);
	// Load param weight_viapoints
	if (!private_nh->getParam("teb_weights/weight_viapoints", tebconfig.optim.weight_viapoint))
	{
		tebconfig.optim.weight_viapoint = teb_weights::DEFAULT_WEIGHT_VIAPOINTS;
	}
	ROS_INFO_STREAM("Using teb_weights/weight_viapoints:=" << tebconfig.optim.weight_viapoint);
	// Load param weight_optimaltime
	if (!private_nh->getParam("teb_weights/weight_optimaltime", tebconfig.optim.weight_optimaltime))
	{
		tebconfig.optim.weight_optimaltime = teb_weights::DEFAULT_WEIGHT_OPTIMALTIME;
	}
	ROS_INFO_STREAM("Using teb_weights/weight_optimaltime:=" << tebconfig.optim.weight_optimaltime);
	// Load param weight_shortest_path
	if (!private_nh->getParam("teb_weights/weight_shortest_path", tebconfig.optim.weight_shortest_path))
	{
		tebconfig.optim.weight_shortest_path = teb_weights::DEFAULT_WEIGHT_SHORTEST_PATH;
	}
	ROS_INFO_STREAM("Using teb_weights/weight_shortest_path:=" << tebconfig.optim.weight_shortest_path);
	// Load param weight_kinematics_turning_radius
	if (!private_nh->getParam("teb_weights/weight_kinematics_turning_radius", tebconfig.optim.weight_kinematics_turning_radius))
	{
		tebconfig.optim.weight_kinematics_turning_radius = teb_weights::DEFAULT_WEIGHT_KINEMATICS_TURNING_RADIUS;
	}
	ROS_INFO_STREAM("Using teb_weights/weight_kinematics_turning_radius:=" << tebconfig.optim.weight_kinematics_turning_radius);
	// Load param weight_kinematics_forward_drive
	if (!private_nh->getParam("teb_weights/weight_kinematics_forward_drive", tebconfig.optim.weight_kinematics_forward_drive))
	{
		tebconfig.optim.weight_kinematics_forward_drive = teb_weights::DEFAULT_WEIGHT_KINEMATICS_FORWARD_DRIVE;
	}
	ROS_INFO_STREAM("Using teb_weights/weight_kinematics_forward_drive:=" << tebconfig.optim.weight_kinematics_forward_drive);
	// Load param weight_kinematics_nh
	if (!private_nh->getParam("teb_weights/weight_kinematics_nh", tebconfig.optim.weight_kinematics_nh))
	{
		tebconfig.optim.weight_kinematics_nh = teb_weights::DEFAULT_WEIGHT_KINEMATICS_NH;
	}
	ROS_INFO_STREAM("Using teb_weights/weight_kinematics_nh:=" << tebconfig.optim.weight_kinematics_nh);
	// Load param weight_max_vel_x
	if (!private_nh->getParam("teb_weights/weight_max_vel_x", tebconfig.optim.weight_max_vel_x))
	{
		tebconfig.optim.weight_max_vel_x = teb_weights::DEFAULT_WEIGHT_MAX_VEL_X;
	}
	ROS_INFO_STREAM("Using teb_weights/weight_max_vel_x:=" << tebconfig.optim.weight_max_vel_x);
	// Load param weight_max_vel_y
	if (!private_nh->getParam("teb_weights/weight_max_vel_y", tebconfig.optim.weight_max_vel_y))
	{
		tebconfig.optim.weight_max_vel_y = teb_weights::DEFAULT_WEIGHT_MAX_VEL_Y;
	}
	ROS_INFO_STREAM("Using teb_weights/weight_max_vel_y:=" << tebconfig.optim.weight_max_vel_y);
	// Load param penalty_epsilon
	if (!private_nh->getParam("teb_weights/penalty_epsilon", tebconfig.optim.penalty_epsilon))
	{
		tebconfig.optim.penalty_epsilon = teb_weights::DEFAULT_PENALTY_EPSILON;
	}
	ROS_INFO_STREAM("Using teb_weights/penalty_epsilon:=" << tebconfig.optim.penalty_epsilon);
	// Load param outer_iterations
	if (!private_nh->getParam("teb_weights/outer_iterations", tebconfig.optim.no_outer_iterations))
	{
		tebconfig.optim.no_outer_iterations = teb_weights::DEFAULT_OUTER_ITERATIONS;
	}
	ROS_INFO_STREAM("Using teb_weights/outer_iterations:=" << tebconfig.optim.no_outer_iterations);
	// Load param inner_iterations
	if (!private_nh->getParam("teb_weights/inner_iterations", tebconfig.optim.no_inner_iterations))
	{
		tebconfig.optim.no_inner_iterations = teb_weights::DEFAULT_INNER_ITERATIONS;
	}
	ROS_INFO_STREAM("Using teb_weights/inner_iterations:=" << tebconfig.optim.no_inner_iterations);
}

}
