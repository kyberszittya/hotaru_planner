/*
 * hotaru_teb_local_planner.hpp
 *
 *  Created on: Apr 1, 2020
 *      Author: kyberszittya
 */

#ifndef INCLUDE_HOTARU_PLANNER_NODE_TEB_HOTARU_TEB_LOCAL_PLANNER_HPP_
#define INCLUDE_HOTARU_PLANNER_NODE_TEB_HOTARU_TEB_LOCAL_PLANNER_HPP_

#include <hotaru_planner_node_teb/hotaru_teb_parameters.hpp>

#include <hotaru_node_elements/hotaru_planner_node_ros1.hpp>
#include <hotaru_node_elements/state_machine/trajectory_statemachine_guard.hpp>

#include <teb_local_planner/teb_local_planner_ros.h>

#include <hotaru_node_elements/trajectory_slicer_strategy.hpp>
#include <hotaru_node_elements/trajectory_lookahead.hpp>

#include <chrono>

#include <grid_map_msgs/GridMap.h>

namespace hotaru
{

class HotaruTebLocalPlannerNode: public HotaruPlannerNodeRos
{
private:
protected:
	std::vector<teb_local_planner::ObstaclePtr> obst_vector;
	teb_local_planner::TebConfig tebconfig;
	teb_local_planner::TebVisualizationPtr visual;
	teb_local_planner::TebOptimalPlannerPtr planner;
	teb_local_planner::RobotFootprintModelPtr robot_model;
	teb_local_planner::ViaPointContainer via_points;
	teb_local_planner::PoseSE2 se2_current_pose;
	teb_local_planner::PoseSE2 se2_plan_end;
	std::vector<teb_local_planner::TrajectoryPointMsg> trajectory_profile;
	std::shared_ptr<LocalPlannerStateMachine> sm_behav_planner;
	std::shared_ptr<SyncGuardLocalPlanner> guard_local_planner;
	// Select a trajectory slicing method
	std::unique_ptr<SpeedLookaheadSlicer> trajectoryslicestrategy;
	// TF handle it in one another thread
	ros::Timer tf_timer;
	ros::Timer timer_trajectory; 									///< Trajectory slice
	ros::Timer timer_trajectory_publish;							///< Publish planned trajectory periodically
	ros::Timer timer_obstacle_update;								///< Update obstacle
	//

	// TODO: place it to an ancestor
	void setSyncStateMachineCallbacks()
	{
		sync_state_machine->setCbAllStateMessageReceived(std::bind(&HotaruTebLocalPlannerNode::startAfterSynchronization, this));
		sync_state_machine->setCbTimeOut(std::bind(&HotaruTebLocalPlannerNode::timeoutSynchronization, this));
	}

	virtual void startAfterSynchronization()
	{
		sm_behav_planner->start();
		timer_planner = nh->createTimer(ros::Duration(1.0/10.0), &HotaruTebLocalPlannerNode::plannerCycle, this);
		timer_planner.start();
		timer_trajectory_publish = nh->createTimer(ros::Duration(1.0/20.0), &HotaruTebLocalPlannerNode::cbTrajectoryPublishTimer, this);
		timer_trajectory_publish.start();
		timer_obstacle_update = nh->createTimer(ros::Duration(1.0/15.0), &HotaruTebLocalPlannerNode::cbObstacleTimer, this);
		ROS_INFO("START PLANNER AFTER SYNC");
	}

	virtual void timeoutSynchronization()
	{
		ROS_INFO("TIMEout\n");
		timer_planner.stop();
		//planner->clearPlanner();
	}

public:
	HotaruTebLocalPlannerNode(std::shared_ptr<ros::NodeHandle> nh,
			std::shared_ptr<ros::NodeHandle> private_nh,
			std::string base_frame, std::string target_frame,
			const bool debug=false):
		HotaruPlannerNodeRos(nh, private_nh, base_frame, target_frame){}

	virtual void plannerCycle(const ros::TimerEvent& e)
	{
		plancycle();
	}

	void cbTfTimer(const ros::TimerEvent& e)
	{
		tf_planner_state.getStampedTransform(transform_base, inv_transform_base);
		geometry_msgs::Pose p0;
		tf_planner_state.getPoseInFrame(pubsubstate->msg_port_current_pose.pose,
				this->transform_base, p0);
		se2_current_pose = teb_local_planner::PoseSE2(p0);
	}

	void cbTrajectoryPublishTimer(const ros::TimerEvent& e)
	{
		pubsubstate->msg_port_refined_trajectory.waypoints.clear();
		mtx_trajectory_update.lock();
		// Don't publish anything until no valid trajectory is received
		if (trajectory_profile.size() > 0)
		{
			for (int i = 0; i < trajectory_profile.size(); i++)
			{
				hotaru_msgs::Waypoint w0;
				tf2::doTransform(trajectory_profile[i].pose, w0.pose.pose, inv_transform_base);
				//w0.twist.twist = v.velocity;
				w0.twist.twist.linear.x = 5.0;
				pubsubstate->msg_port_refined_trajectory.waypoints.emplace_back(w0);
			}
			mtx_trajectory_update.unlock();
			//
			for (int i = trajectoryslicestrategy->getSlicePoint();
				i < std::min(trajectoryslicestrategy->getSlicePoint() +
						hotaru::TrajectoryLookaheadPoint::lookaheadPoint(
							10.0, pubsubstate->msg_port_current_velocity.twist),
							static_cast<int>(pubsubstate->msg_port_input_trajectory.waypoints.size())); i++)
			{
				pubsubstate->msg_port_refined_trajectory.waypoints.emplace_back(
					pubsubstate->msg_port_input_trajectory.waypoints[i]
				);
			}

			pubsubstate->msg_port_refined_trajectory.header.stamp = ros::Time::now();
			this->port_refined_trajectory.publish(pubsubstate->msg_port_refined_trajectory);
		}
	}

	virtual void config()
	{
		tebconfig.map_frame = tf_planner_state.getBaseFrame();
		//
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
		if (!private_nh->getParam("min_turning_radius", tebconfig.robot.min_turning_radius))
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
		if (!private_nh->getParam("wheelbase", tebconfig.robot.wheelbase))
		{
			tebconfig.robot.wheelbase = hotaru::teb::DEFAULT_WHEELBASE;
		}
		tebconfig.robot.cmd_angle_instead_rotvel = true;
		// Set minimal obstacle distance
		if (!private_nh->getParam("min_obstacle_distance", tebconfig.obstacles.min_obstacle_dist))
		{
			tebconfig.obstacles.min_obstacle_dist = hotaru::teb::DEFAULT_MIN_OBSTACLE_DISTANCE;
		}
		ROS_INFO_STREAM("Using minimum obstacle distance: " << tebconfig.obstacles.min_obstacle_dist);
		// Set minimal inflation distance
		if (!private_nh->getParam("inflation_distance", tebconfig.obstacles.min_obstacle_dist))
		{
			tebconfig.obstacles.inflation_dist = hotaru::teb::DEFAULT_MIN_INFLATION;
		}
		ROS_INFO_STREAM("Using inflation distance: " << tebconfig.obstacles.inflation_dist);
		// Set minimal weight obstacle
		if (!private_nh->getParam("weight_obstacle", tebconfig.optim.weight_obstacle))
		{
			tebconfig.optim.weight_obstacle = hotaru::teb::DEFAULT_WEIGHT_OBSTACLE;
		}
		ROS_INFO_STREAM("Weight obstacle: " << tebconfig.optim.weight_obstacle);
		//
		if (!private_nh->getParam("weight_viapoints", tebconfig.optim.weight_viapoint))
		{
			tebconfig.optim.weight_viapoint = hotaru::teb::DEFAULT_WEIGHT_VIAPOINTS;
		}
		ROS_INFO_STREAM("Using weight viapoints: " << tebconfig.optim.weight_viapoint);
		//
		tebconfig.optim.weight_max_vel_y = 0;
		// Load: weight kinematics turning radius
		if (!private_nh->getParam("weight_kinematics_turning_radius", tebconfig.optim.weight_kinematics_turning_radius))
		{
			tebconfig.optim.weight_kinematics_turning_radius = hotaru::teb::DEFAULT_WEIGHT_KINEMATICS_TURNING_RADIUS;
		}
		ROS_INFO_STREAM("Using weight turning radius (kinematics): " << tebconfig.optim.weight_viapoint);
		// Load: weight forward drive
		if (!private_nh->getParam("weight_kinematics_forward_drive", tebconfig.optim.weight_kinematics_forward_drive))
		{
			tebconfig.optim.weight_kinematics_forward_drive = hotaru::teb::DEFAULT_WEIGHT_KINEMATICS_TURNING_RADIUS;
		}
		ROS_INFO_STREAM("Using weight forward drive: " << tebconfig.optim.weight_kinematics_forward_drive);
		// Load: nh kinematics
		if (!private_nh->getParam("weight_kinematics_nh", tebconfig.optim.weight_kinematics_nh))
		{
			tebconfig.optim.weight_kinematics_nh = hotaru::teb::DEFAULT_WEIGHT_KINEMATICS_NH;
		}
		ROS_INFO_STREAM("Using weight kinematics nh: " << tebconfig.optim.weight_kinematics_nh);
		// Load: xy goal tolerance
		if (!private_nh->getParam("xy_goal_tolerance", tebconfig.goal_tolerance.xy_goal_tolerance))
		{
			tebconfig.goal_tolerance.xy_goal_tolerance = hotaru::teb::DEFAULT_XY_GOAL_TOLERANCE;
		}
		ROS_INFO_STREAM("Using xy goal tolerance: " << tebconfig.goal_tolerance.xy_goal_tolerance);
		// Load: yaw goal tolerance
		if (!private_nh->getParam("yaw_goal_tolerance", tebconfig.goal_tolerance.yaw_goal_tolerance))
		{
			tebconfig.goal_tolerance.yaw_goal_tolerance = hotaru::teb::DEFAULT_YAW_GOAL_TOLERANCE;
		}
		ROS_INFO_STREAM("Using yaw goal tolerance: " << tebconfig.goal_tolerance.yaw_goal_tolerance);
	}

	virtual void executeUpdate_velocity() override
	{
		const double speed = sqrt(
				(pubsubstate->msg_port_current_velocity.twist.linear.x*pubsubstate->msg_port_current_velocity.twist.linear.x)+
				(pubsubstate->msg_port_current_velocity.twist.linear.y*pubsubstate->msg_port_current_velocity.twist.linear.y));
		trajectoryslicestrategy->setSpeed(speed);
	}

	virtual void execute_update_input_trajectory() override
	{

	}

	virtual void executeUpdate_closest_waypoint() override
	{
		trajectoryslicestrategy->setClosestWaypointIndex(pubsubstate->msg_port_closests_waypoint.data);
	}

	virtual void executeReplan_request_sig() override
	{
		if (pubsubstate->msg_port_replan_request_sig.eval)
		{
			sm_behav_planner->propagateSignal(
				std::make_shared<hotaru::trajectory_signals::SignalReplanningTrajectory>(
						pubsubstate->msg_port_replan_request_sig.header.stamp.toNSec())
			);
			sm_behav_planner->stepstatemachine();
		}

	}

	virtual void execute_update_current_pose() override
	{

	}

	void onReplan()
	{
		ROS_INFO("RUNAWAY");
	}

	void onStart()
	{

	}

	void onWaiting()
	{

	}

	void onRelay()
	{

	}

	void cbObstacleTimer(const ros::TimerEvent& e)
	{
		mtx_planner.lock();
		obst_vector.clear();
		using namespace teb_local_planner;

		for (const auto o: pubsubstate->msg_port_poly_obstacle.obstacles)
		{
			obst_vector.push_back(ObstaclePtr(new CircularObstacle(o.pose.position.x, o.pose.position.y, o.radius)));
		}
		mtx_planner.unlock();
	}

	virtual bool initPre() override
	{
		return true;
	}



	bool isPoseStateValid()
	{
		// Check
		if (!tf_planner_state.getStampedTransform(transform_base, inv_transform_base)) return false;
		geometry_msgs::Pose p0;
		if (pubsubstate->msg_port_input_trajectory.waypoints.size() > 0)
		{
			tf_planner_state.getPoseInFrame(pubsubstate->msg_port_input_trajectory.waypoints[0].pose.pose, transform_base, p0);
			if (rei::isInvalidPoint(p0.position)) return false;
		}
		else
		{
			return false;
		}
		tf_planner_state.getPoseInFrame(pubsubstate->msg_port_current_pose.pose, transform_base, p0);
		// Check if pose is valid
		if (rei::isInvalidPoint(p0.position))
		{
			return false;
		}
		return true;
	}

	virtual void executeUpdate_obstacles() override
	{

	}

	virtual bool assignSyncGuards() override
	{
		sync_guard->setCrispGuardState_Start(std::bind(&HotaruTebLocalPlannerNode::isPoseStateValid, this));
		return true;
	}

	virtual bool initPost() override
	{
		pubsubstate->msg_port_refined_trajectory.header.frame_id = "map";
		// Trajectory slicing
		trajectoryslicestrategy = std::unique_ptr<SpeedLookaheadSlicer>(new SpeedLookaheadSlicer(3.5, 25.0)); // minimum lookahead distance is 12.0 meters in case of trajectory slicing
		if (trajectoryslicestrategy == nullptr)
		{
			return false;
		}
		// Initialize ros publishers
		using namespace teb_local_planner;
		config();
		visual = TebVisualizationPtr(new TebVisualization(*nh, tebconfig));
		// TODO: state machine callback to ancestor class
		setSyncStateMachineCallbacks();
		//  Setup planner SM callbacks
		guard_local_planner = std::make_shared<SyncGuardLocalPlanner>(sync_state_machine);
		sm_behav_planner = std::make_shared<LocalPlannerStateMachine>(notifier, guard_local_planner);
		sm_behav_planner->setCbRelay(std::bind(&HotaruTebLocalPlannerNode::onRelay, this));
		sm_behav_planner->setCbReplanningEnter(std::bind(&HotaruTebLocalPlannerNode::onReplan, this));
		sm_behav_planner->setCbWaiting(std::bind(&HotaruTebLocalPlannerNode::onWaiting, this));
		//
		if (visual == nullptr)
		{
			return false;
		}
		robot_model = RobotFootprintModelPtr(new TwoCirclesRobotFootprint(2.0,1.5, 2.0, 1.5));
		planner = TebOptimalPlannerPtr(new TebOptimalPlanner(
				tebconfig, &obst_vector, robot_model, visual, nullptr));
		// Start TF thread
		tf_timer = nh->createTimer(ros::Duration(1.0/40.0), &HotaruTebLocalPlannerNode::cbTfTimer, this);
		tf_timer.start();
		timer_trajectory = nh->createTimer(ros::Duration(0.1), &HotaruTebLocalPlannerNode::cbTrajectorySlicer, this);
		timer_trajectory.start();
		return true;
	}

	void cbTrajectorySlicer(const ros::TimerEvent& e)
	{
		trajectoryslicestrategy->calcSlicePointIndex(pubsubstate->msg_port_input_trajectory, transform_base);
		if (trajectoryslicestrategy->getSlicePoint() >= 0)
		{
			// TODO: something is WTF with slicing the trajectory
			slicer.sliceTrajectory(pubsubstate->msg_port_input_trajectory, replanned_trajectory, waypoint_original, trajectoryslicestrategy->getSlicePoint());
			geometry_msgs::Pose p_end;
			tf_planner_state.getPoseInFrame(
					pubsubstate->msg_port_input_trajectory.waypoints[trajectoryslicestrategy->getSlicePoint()].pose.pose,
					this->transform_base, p_end);
			se2_plan_end = teb_local_planner::PoseSE2(p_end);
			mtx_planner.lock();
			via_points.clear();
			geometry_msgs::Point p;
			geometry_msgs::Point p_Wro;
			int _via_lookaheadpoint = TrajectoryLookaheadPoint::lookaheadPoint(1.5, pubsubstate->msg_port_current_velocity.twist);
			// There are some points that are definetely not needed
			for (
				int i = pubsubstate->msg_port_closests_waypoint.data+_via_lookaheadpoint;
					i < trajectoryslicestrategy->getSlicePoint(); i++)
			{
				p = pubsubstate->msg_port_input_trajectory.waypoints[i].pose.pose.position;
				tf2::doTransform(p, p_Wro, this->transform_base);
				via_points.push_back(
					std::move(Eigen::Vector2d(p_Wro.x, p_Wro.y))
				);
			}
			planner->setViaPoints(&via_points);
			mtx_planner.unlock();
		}

	}


	virtual bool plancycle()
	{
		if (trajectoryslicestrategy->getSlicePoint()<0) return false;
		// TODO add valid tf as synchronization requirement
		if (std::isnan(se2_plan_end.x()) ||
				std::isnan(se2_plan_end.y()) ||
				std::isnan(se2_plan_end.theta()))
		{
			ROS_ERROR("Invalid goal point selected");
			return false;
		}
		mtx_planner.lock();
		auto t_start = std::chrono::steady_clock::now();
		bool plan_success = planner->plan(se2_current_pose, se2_plan_end);
		auto t_end = std::chrono::steady_clock::now();
		mtx_planner.unlock();
		std::chrono::duration<double> elps = t_end-t_start;
		if (plan_success)
		{
			mtx_trajectory_update.lock();
			try
			{
				planner->getFullTrajectory(trajectory_profile);
			}
			catch(std::runtime_error &e)
			{
				ROS_ERROR("Replan error!");
			}
			mtx_trajectory_update.unlock();
		}
		if (pubsubstate->debug)
		{
			planner->visualize();
			pubsubstate->msg_port_calc_planner_time.data = elps.count();
			port_calc_planner_time.publish(pubsubstate->msg_port_calc_planner_time);
		}
		std::cout << se2_current_pose << '\t' <<  se2_plan_end << '\t' << elps.count() << '\t' << trajectoryslicestrategy->getSlicePoint() << '\n';
		return true;
	}

};

}
#endif /* INCLUDE_HOTARU_PLANNER_NODE_TEB_HOTARU_TEB_LOCAL_PLANNER_HPP_ */

