/*
 * hotaru_teb_local_planner.hpp
 *
 *  Created on: Apr 1, 2020
 *      Author: kyberszittya
 */

#ifndef INCLUDE_HOTARU_PLANNER_NODE_TEB_HOTARU_TEB_LOCAL_PLANNER_HPP_
#define INCLUDE_HOTARU_PLANNER_NODE_TEB_HOTARU_TEB_LOCAL_PLANNER_HPP_

#include <hotaru_node_elements/hotaru_planner_node_ros1.hpp>
#include <hotaru_node_elements/state_machine/trajectory_statemachine_guard.hpp>

#include <teb_local_planner/teb_local_planner_ros.h>

#include <hotaru_node_elements/trajectory_slicer_strategy.hpp>

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
			std::string base_frame, std::string target_frame,
			const bool debug=false):
		HotaruPlannerNodeRos(nh, base_frame, target_frame, debug){}

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
		mtx_trajectory_update.lock();
		// Skip the first point
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
				i < std::min(trajectoryslicestrategy->getSlicePoint() + 70,
						static_cast<int>(pubsubstate->msg_port_input_trajectory.waypoints.size())); i++)
		{
			pubsubstate->msg_port_refined_trajectory.waypoints.emplace_back(
				pubsubstate->msg_port_input_trajectory.waypoints[i]
			);
		}

		pubsubstate->msg_port_refined_trajectory.header.stamp = ros::Time::now();
		this->port_refined_trajectory.publish(pubsubstate->msg_port_refined_trajectory);
	}

	virtual void config()
	{
		tebconfig.map_frame = tf_planner_state.getBaseFrame();
		//
		tebconfig.trajectory.dt_ref = 1.5;
		tebconfig.trajectory.dt_hysteresis = 0.4;
		//
		tebconfig.robot.min_turning_radius = 10.4;
		tebconfig.robot.acc_lim_theta = 0.05;
		tebconfig.robot.acc_lim_x = 0.2;
		tebconfig.robot.max_vel_x_backwards = 0.01;
		tebconfig.robot.wheelbase = 2.7;
		tebconfig.robot.cmd_angle_instead_rotvel = true;
		//
		tebconfig.obstacles.min_obstacle_dist = 1.5;
		tebconfig.optim.weight_obstacle = 200;
		tebconfig.optim.weight_viapoint = 15;
		tebconfig.optim.weight_max_vel_y = 0;
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

	virtual void execute_update_current_pose() override
	{

	}

	void onReplan()
	{

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
			// There are some points that are definetely not needed
			for (int i = pubsubstate->msg_port_closests_waypoint.data+15; i < trajectoryslicestrategy->getSlicePoint(); i++)
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
			pubsubstate->msg_port_refined_trajectory.waypoints.clear();
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
