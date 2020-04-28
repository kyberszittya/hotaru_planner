#include <hotaru_planner_node_teb/hotaru_teb_local_planner.hpp>

namespace hotaru
{

void HotaruTebLocalPlannerNode::config()
{
	tebconfig.map_frame = tf_planner_state.getBaseFrame();
	//

	if (!private_nh->getParam("robot/min_turning_radius", tebconfig.robot.min_turning_radius))
	{
		tebconfig.robot.min_turning_radius = hotaru::teb::DEFAULT_MIN_TURNING_RADIUS;
	}
	ROS_INFO_STREAM("Using minimum turning radius: " << tebconfig.robot.min_turning_radius);
	tebconfig.robot.max_vel_y = 0.0;
	tebconfig.robot.max_vel_x_backwards = 0.0;
	//
	tebconfig.robot.acc_lim_theta = 0.05;
	tebconfig.robot.acc_lim_x = 0.2;
	tebconfig.robot.max_vel_x_backwards = 0.01;
	if (!private_nh->getParam("robot/wheelbase", tebconfig.robot.wheelbase))
	{
		tebconfig.robot.wheelbase = hotaru::teb::DEFAULT_WHEELBASE;
	}
	tebconfig.robot.cmd_angle_instead_rotvel = true;
	if (!private_nh->getParam("dt_ref", tebconfig.trajectory.dt_ref))
	{
		tebconfig.trajectory.dt_ref = hotaru::teb::DEFAULT_TEB_CONFIG_TRAJECTORY_DT_REF;
	}
	ROS_INFO_STREAM("Using dt ref: " << tebconfig.trajectory.dt_ref);
	//
	if (!private_nh->getParam("dt_hysteresis", tebconfig.trajectory.dt_hysteresis))
	{
		tebconfig.trajectory.dt_hysteresis = hotaru::teb::DEFAULT_TEB_CONFIG_TRAJECTORY_DT_HYSTERESIS;
	}
	ROS_INFO_STREAM("Using dt hysteresis: " << tebconfig.trajectory.dt_hysteresis);
	//
	// Set minimal obstacle distance
	if (!private_nh->getParam("teb_trajectory/min_obstacle_distance", tebconfig.obstacles.min_obstacle_dist))
	{
		tebconfig.obstacles.min_obstacle_dist = hotaru::teb::DEFAULT_MIN_OBSTACLE_DISTANCE;
	}
	ROS_INFO_STREAM("Using minimum obstacle distance: " << tebconfig.obstacles.min_obstacle_dist);
	// Set minimal inflation distance
	if (!private_nh->getParam("teb_trajectory/inflation_distance", tebconfig.obstacles.min_obstacle_dist))
	{
		tebconfig.obstacles.inflation_dist = hotaru::teb::DEFAULT_MIN_INFLATION;
	}
	ROS_INFO_STREAM("Using inflation distance: " << tebconfig.obstacles.inflation_dist);
	// Load: xy goal tolerance
	if (!private_nh->getParam("teb_trajectory/xy_goal_tolerance", tebconfig.goal_tolerance.xy_goal_tolerance))
	{
		tebconfig.goal_tolerance.xy_goal_tolerance = hotaru::teb::DEFAULT_XY_GOAL_TOLERANCE;
	}
	ROS_INFO_STREAM("Using xy goal tolerance: " << tebconfig.goal_tolerance.xy_goal_tolerance);
	// Load: yaw goal tolerance
	if (!private_nh->getParam("teb_trajectory/yaw_goal_tolerance", tebconfig.goal_tolerance.yaw_goal_tolerance))
	{
		tebconfig.goal_tolerance.yaw_goal_tolerance = hotaru::teb::DEFAULT_YAW_GOAL_TOLERANCE;
	}
	ROS_INFO_STREAM("Using yaw goal tolerance: " << tebconfig.goal_tolerance.yaw_goal_tolerance);
	// Optimization weights
	// Set minimal weight obstacle
	if (!private_nh->getParam("teb_weights/weight_obstacle", tebconfig.optim.weight_obstacle))
	{
		tebconfig.optim.weight_obstacle = hotaru::teb::DEFAULT_WEIGHT_OBSTACLE;
	}
	ROS_INFO_STREAM("Weight obstacle: " << tebconfig.optim.weight_obstacle);
	//
	if (!private_nh->getParam("teb_weights/weight_viapoints", tebconfig.optim.weight_viapoint))
	{
		tebconfig.optim.weight_viapoint = hotaru::teb::DEFAULT_WEIGHT_VIAPOINTS;
	}
	ROS_INFO_STREAM("Using weight viapoints: " << tebconfig.optim.weight_viapoint);
	// Set weight optimaltime
	if (!private_nh->getParam("teb_weights/weight_optimaltime", tebconfig.optim.weight_optimaltime))
	{
		tebconfig.optim.weight_optimaltime = hotaru::teb::DEFAULT_WEIGHT_OPTIMALTIME;
	}
	ROS_INFO_STREAM("Using weight optimal time: " << tebconfig.optim.weight_optimaltime);
	// Set weight shortest path
	if (!private_nh->getParam("teb_weights/weight_shortest_path", tebconfig.optim.weight_shortest_path))
	{
		tebconfig.optim.weight_shortest_path = hotaru::teb::DEFAULT_WEIGHT_SHORTEST_PATH;
	}
	ROS_INFO_STREAM("Using weight shortest path: " << tebconfig.optim.weight_shortest_path);
	//
	tebconfig.optim.weight_max_vel_y = 0;
	// Load: weight kinematics turning radius
	if (!private_nh->getParam("teb_weights/weight_kinematics_turning_radius", tebconfig.optim.weight_kinematics_turning_radius))
	{
		tebconfig.optim.weight_kinematics_turning_radius = hotaru::teb::DEFAULT_WEIGHT_KINEMATICS_TURNING_RADIUS;
	}
	ROS_INFO_STREAM("Using weight turning radius (kinematics): " << tebconfig.optim.weight_viapoint);
	// Load: weight forward drive
	if (!private_nh->getParam("teb_weights/weight_kinematics_forward_drive", tebconfig.optim.weight_kinematics_forward_drive))
	{
		tebconfig.optim.weight_kinematics_forward_drive = hotaru::teb::DEFAULT_WEIGHT_KINEMATICS_FORWARD_DRIVE;
	}
	ROS_INFO_STREAM("Using weight forward drive: " << tebconfig.optim.weight_kinematics_forward_drive);
	// Load: nh kinematics
	if (!private_nh->getParam("teb_weights/weight_kinematics_nh", tebconfig.optim.weight_kinematics_nh))
	{
		tebconfig.optim.weight_kinematics_nh = hotaru::teb::DEFAULT_WEIGHT_KINEMATICS_NH;
	}
	ROS_INFO_STREAM("Using weight kinematics nh: " << tebconfig.optim.weight_kinematics_nh);

}

}
