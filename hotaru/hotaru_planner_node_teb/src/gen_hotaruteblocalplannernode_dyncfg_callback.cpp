#include <hotaru_planner_node_teb/hotaruteblocalplannernode.hpp>

namespace hotaru
{

void HotaruTebLocalPlannerNode::callbackReconfigure(hotaru_planner_node_teb::HotaruTebLocalPlannerNodeConfig & config,uint32_t level)
{
	if (planner_config.via_points_speed_linear_ratio != config.via_points_speed_linear_ratio)
	{
		ROS_INFO_STREAM("Updated message [planner_config.via_points_speed_linear_ratio]: via_points_speed_linear_ratio" << config.via_points_speed_linear_ratio);
	}
	    planner_config.via_points_speed_linear_ratio = config.via_points_speed_linear_ratio;
	if (tebconfig.robot.wheelbase != config.wheelbase)
	{
		ROS_INFO_STREAM("Updated message [tebconfig.robot.wheelbase]: wheelbase" << config.wheelbase);
	}
	    tebconfig.robot.wheelbase = config.wheelbase;
	if (tebconfig.robot.min_turning_radius != config.min_turning_radius)
	{
		ROS_INFO_STREAM("Updated message [tebconfig.robot.min_turning_radius]: min_turning_radius" << config.min_turning_radius);
	}
	    tebconfig.robot.min_turning_radius = config.min_turning_radius;
	if (tebconfig.robot.max_vel_x != config.max_vel_x)
	{
		ROS_INFO_STREAM("Updated message [tebconfig.robot.max_vel_x]: max_vel_x" << config.max_vel_x);
	}
	    tebconfig.robot.max_vel_x = config.max_vel_x;
	if (tebconfig.robot.max_vel_y != config.max_vel_y)
	{
		ROS_INFO_STREAM("Updated message [tebconfig.robot.max_vel_y]: max_vel_y" << config.max_vel_y);
	}
	    tebconfig.robot.max_vel_y = config.max_vel_y;
	if (tebconfig.robot.acc_lim_theta != config.acc_lim_theta)
	{
		ROS_INFO_STREAM("Updated message [tebconfig.robot.acc_lim_theta]: acc_lim_theta" << config.acc_lim_theta);
	}
	    tebconfig.robot.acc_lim_theta = config.acc_lim_theta;
	if (tebconfig.robot.acc_lim_x != config.acc_lim_x)
	{
		ROS_INFO_STREAM("Updated message [tebconfig.robot.acc_lim_x]: acc_lim_x" << config.acc_lim_x);
	}
	    tebconfig.robot.acc_lim_x = config.acc_lim_x;
	if (tebconfig.robot.max_vel_x_backwards != config.max_vel_x_backwards)
	{
		ROS_INFO_STREAM("Updated message [tebconfig.robot.max_vel_x_backwards]: max_vel_x_backwards" << config.max_vel_x_backwards);
	}
	    tebconfig.robot.max_vel_x_backwards = config.max_vel_x_backwards;
	if (tebconfig.robot.cmd_angle_instead_rotvel != config.cmd_angle_instead_rotvel)
	{
		ROS_INFO_STREAM("Updated message [tebconfig.robot.cmd_angle_instead_rotvel]: cmd_angle_instead_rotvel" << config.cmd_angle_instead_rotvel);
	}
	    tebconfig.robot.cmd_angle_instead_rotvel = config.cmd_angle_instead_rotvel;
	if (tebconfig.hcp.enable_homotopy_class_planning != config.homotopy_enabled)
	{
		ROS_INFO_STREAM("Updated message [tebconfig.hcp.enable_homotopy_class_planning]: homotopy_enabled" << config.homotopy_enabled);
	}
	    tebconfig.hcp.enable_homotopy_class_planning = config.homotopy_enabled;
	if (tebconfig.trajectory.dt_ref != config.dt_ref)
	{
		ROS_INFO_STREAM("Updated message [tebconfig.trajectory.dt_ref]: dt_ref" << config.dt_ref);
	}
	    tebconfig.trajectory.dt_ref = config.dt_ref;
	if (tebconfig.trajectory.dt_hysteresis != config.dt_hysteresis)
	{
		ROS_INFO_STREAM("Updated message [tebconfig.trajectory.dt_hysteresis]: dt_hysteresis" << config.dt_hysteresis);
	}
	    tebconfig.trajectory.dt_hysteresis = config.dt_hysteresis;
	if (tebconfig.obstacles.min_obstacle_dist != config.min_obstacle_distance)
	{
		ROS_INFO_STREAM("Updated message [tebconfig.obstacles.min_obstacle_dist]: min_obstacle_distance" << config.min_obstacle_distance);
	}
	    tebconfig.obstacles.min_obstacle_dist = config.min_obstacle_distance;
	if (tebconfig.obstacles.inflation_dist != config.inflation_distance)
	{
		ROS_INFO_STREAM("Updated message [tebconfig.obstacles.inflation_dist]: inflation_distance" << config.inflation_distance);
	}
	    tebconfig.obstacles.inflation_dist = config.inflation_distance;
	if (tebconfig.goal_tolerance.xy_goal_tolerance != config.xy_goal_tolerance)
	{
		ROS_INFO_STREAM("Updated message [tebconfig.goal_tolerance.xy_goal_tolerance]: xy_goal_tolerance" << config.xy_goal_tolerance);
	}
	    tebconfig.goal_tolerance.xy_goal_tolerance = config.xy_goal_tolerance;
	if (tebconfig.goal_tolerance.yaw_goal_tolerance != config.yaw_goal_tolerance)
	{
		ROS_INFO_STREAM("Updated message [tebconfig.goal_tolerance.yaw_goal_tolerance]: yaw_goal_tolerance" << config.yaw_goal_tolerance);
	}
	    tebconfig.goal_tolerance.yaw_goal_tolerance = config.yaw_goal_tolerance;
	if (tebconfig.optim.no_inner_iterations != config.no_inner_iterations)
	{
		ROS_INFO_STREAM("Updated message [tebconfig.optim.no_inner_iterations]: no_inner_iterations" << config.no_inner_iterations);
	}
	    tebconfig.optim.no_inner_iterations = config.no_inner_iterations;
	if (tebconfig.optim.no_outer_iterations != config.no_outer_iterations)
	{
		ROS_INFO_STREAM("Updated message [tebconfig.optim.no_outer_iterations]: no_outer_iterations" << config.no_outer_iterations);
	}
	    tebconfig.optim.no_outer_iterations = config.no_outer_iterations;
	if (tebconfig.optim.penalty_epsilon != config.penalty_epsilon)
	{
		ROS_INFO_STREAM("Updated message [tebconfig.optim.penalty_epsilon]: penalty_epsilon" << config.penalty_epsilon);
	}
	    tebconfig.optim.penalty_epsilon = config.penalty_epsilon;
	if (tebconfig.optim.weight_obstacle != config.weight_obstacle)
	{
		ROS_INFO_STREAM("Updated message [tebconfig.optim.weight_obstacle]: weight_obstacle" << config.weight_obstacle);
	}
	    tebconfig.optim.weight_obstacle = config.weight_obstacle;
	if (tebconfig.optim.weight_inflation != config.weight_inflation)
	{
		ROS_INFO_STREAM("Updated message [tebconfig.optim.weight_inflation]: weight_inflation" << config.weight_inflation);
	}
	    tebconfig.optim.weight_inflation = config.weight_inflation;
	if (tebconfig.optim.weight_viapoint != config.weight_viapoints)
	{
		ROS_INFO_STREAM("Updated message [tebconfig.optim.weight_viapoint]: weight_viapoints" << config.weight_viapoints);
	}
	    tebconfig.optim.weight_viapoint = config.weight_viapoints;
	if (tebconfig.optim.weight_optimaltime != config.weight_optimaltime)
	{
		ROS_INFO_STREAM("Updated message [tebconfig.optim.weight_optimaltime]: weight_optimaltime" << config.weight_optimaltime);
	}
	    tebconfig.optim.weight_optimaltime = config.weight_optimaltime;
	if (tebconfig.optim.weight_shortest_path != config.weight_shortest_path)
	{
		ROS_INFO_STREAM("Updated message [tebconfig.optim.weight_shortest_path]: weight_shortest_path" << config.weight_shortest_path);
	}
	    tebconfig.optim.weight_shortest_path = config.weight_shortest_path;
	if (tebconfig.optim.weight_acc_lim_x != config.weight_acc_lim_x)
	{
		ROS_INFO_STREAM("Updated message [tebconfig.optim.weight_acc_lim_x]: weight_acc_lim_x" << config.weight_acc_lim_x);
	}
	    tebconfig.optim.weight_acc_lim_x = config.weight_acc_lim_x;
	if (tebconfig.optim.weight_acc_lim_y != config.weight_acc_lim_y)
	{
		ROS_INFO_STREAM("Updated message [tebconfig.optim.weight_acc_lim_y]: weight_acc_lim_y" << config.weight_acc_lim_y);
	}
	    tebconfig.optim.weight_acc_lim_y = config.weight_acc_lim_y;
	if (tebconfig.optim.weight_acc_lim_theta != config.weight_acc_lim_theta)
	{
		ROS_INFO_STREAM("Updated message [tebconfig.optim.weight_acc_lim_theta]: weight_acc_lim_theta" << config.weight_acc_lim_theta);
	}
	    tebconfig.optim.weight_acc_lim_theta = config.weight_acc_lim_theta;
	if (tebconfig.optim.weight_max_vel_x != config.weight_max_vel_x)
	{
		ROS_INFO_STREAM("Updated message [tebconfig.optim.weight_max_vel_x]: weight_max_vel_x" << config.weight_max_vel_x);
	}
	    tebconfig.optim.weight_max_vel_x = config.weight_max_vel_x;
	if (tebconfig.optim.weight_max_vel_y != config.weight_max_vel_y)
	{
		ROS_INFO_STREAM("Updated message [tebconfig.optim.weight_max_vel_y]: weight_max_vel_y" << config.weight_max_vel_y);
	}
	    tebconfig.optim.weight_max_vel_y = config.weight_max_vel_y;
	if (tebconfig.optim.weight_max_vel_theta != config.weight_max_vel_theta)
	{
		ROS_INFO_STREAM("Updated message [tebconfig.optim.weight_max_vel_theta]: weight_max_vel_theta" << config.weight_max_vel_theta);
	}
	    tebconfig.optim.weight_max_vel_theta = config.weight_max_vel_theta;
	if (tebconfig.optim.weight_kinematics_turning_radius != config.weight_kinematics_turning_radius)
	{
		ROS_INFO_STREAM("Updated message [tebconfig.optim.weight_kinematics_turning_radius]: weight_kinematics_turning_radius" << config.weight_kinematics_turning_radius);
	}
	    tebconfig.optim.weight_kinematics_turning_radius = config.weight_kinematics_turning_radius;
	if (tebconfig.optim.weight_kinematics_forward_drive != config.weight_kinematics_forward_drive)
	{
		ROS_INFO_STREAM("Updated message [tebconfig.optim.weight_kinematics_forward_drive]: weight_kinematics_forward_drive" << config.weight_kinematics_forward_drive);
	}
	    tebconfig.optim.weight_kinematics_forward_drive = config.weight_kinematics_forward_drive;
	if (tebconfig.optim.weight_kinematics_nh != config.weight_kinematics_nh)
	{
		ROS_INFO_STREAM("Updated message [tebconfig.optim.weight_kinematics_nh]: weight_kinematics_nh" << config.weight_kinematics_nh);
	}
	    tebconfig.optim.weight_kinematics_nh = config.weight_kinematics_nh;
	if (tebconfig.optim.weight_adapt_factor != config.weight_adapt_factor)
	{
		ROS_INFO_STREAM("Updated message [tebconfig.optim.weight_adapt_factor]: weight_adapt_factor" << config.weight_adapt_factor);
	}
	    tebconfig.optim.weight_adapt_factor = config.weight_adapt_factor;
	if (tebconfig.optim.obstacle_cost_exponent != config.obstacle_cost_exponent)
	{
		ROS_INFO_STREAM("Updated message [tebconfig.optim.obstacle_cost_exponent]: obstacle_cost_exponent" << config.obstacle_cost_exponent);
	}
	    tebconfig.optim.obstacle_cost_exponent = config.obstacle_cost_exponent;
}

}
