/*
 * hotaru_local_planner_node.cpp
 *
 *  Created on: Jan 13, 2020
 *      Author: kyberszittya
 */



#include "interfaceros_hotaru_local_planner.h"

#include <teb_local_planner/graph_search.h>
#include <teb_local_planner/teb_local_planner_ros.h>

#include <hotaru_common/planner_components/abstract_planner.hpp>
#include <hotaru_common/state_machine/trajectory_statemachine.hpp>
#include <hotaru_common/state_machine/trajectory_signal.hpp>

#include <rei_common/geometric_utilities.hpp>


constexpr unsigned int PLANNER_SM_HZ = { 40 };
constexpr unsigned int PLANNER_SYNC_HZ = { 100 };

class TebHotaruLocalPlanner: public hotaru::InterfaceRos_Hotarulocalplanner,
	public hotaru::Abstract_RosLocalPlanner
{
private:
	double current_speed;
	double ref_velocity;
protected:
	std::shared_ptr<teb_local_planner::ObstContainer> obstacle_container;
	teb_local_planner::RobotFootprintModelPtr robot_footprint;
	teb_local_planner::TebConfig conf;
	teb_local_planner::TebVisualizationPtr viz;
	std::unique_ptr<teb_local_planner::TebOptimalPlanner> planner;
	std::shared_ptr<teb_local_planner::ViaPointContainer> via_points;
	// Starting plan as pose stamped
	std::vector<teb_local_planner::TrajectoryPointMsg> _full_trajectory;
	// Timers
	ros::Timer timer_cycle;
	ros::Timer timer_planner_sm;
	ros::Timer timer_sync_sm;

	std::mutex plan_source_modification;


	std::vector<geometry_msgs::Pose> transformed_poses_current_pose;



	virtual void executeReconstructWaypoints() override {
		plan_source_modification.lock();
		trajectory_slicer.calcLookaheadIndex(pubsubstate->msg_sub_base_waypoints);
		reconstructStartingPlanPoints(pubsubstate->msg_sub_base_waypoints,
				pubsubstate->msg_sub_current_pose,
				pubsubstate->msg_closest_waypoint.data);
		plan_source_modification.unlock();
	}

	virtual void executePlannerMethods() override {

	}
public:
	TebHotaruLocalPlanner(std::shared_ptr<ros::NodeHandle> nh):
		ref_velocity(0.0),
		InterfaceRos_Hotarulocalplanner(nh){}

	virtual void executeSynchWithPose() override
	{
		plan_source_modification.lock();
		syncTfPose();
		plan_source_modification.unlock();
		//ROS_INFO("synchronize");
	}

	virtual void executeUpdateObstacles() override
	{

		plan_source_modification.lock();
		obstacle_container->clear();

		using namespace teb_local_planner;
		for (auto& marker: pubsubstate->msg_sub_filtered_obstacles.markers)
		{
			switch(marker.type)
			{
				case visualization_msgs::Marker::SPHERE:
				{
					obstacle_container->push_back(ObstaclePtr(new CircularObstacle(
							marker.pose.position.x,
							marker.pose.position.y,
							marker.scale.x/2.0)));
					break;
				}
			}
		}
		plan_source_modification.unlock();

	}

	virtual void executeReplanRequest()
	{
		using namespace hotaru::trajectory_signals;

		std::shared_ptr<rei::AbstractSignalInterface> _sig;
		if (pubsubstate->msg_sub_replan_request_sig.eval)
		{
			// RISING EDGE
			_sig = std::shared_ptr<rei::AbstractSignalInterface>(
				new SignalReplanningTrajectory(
					pubsubstate->msg_sub_replan_request_sig.header.stamp.toNSec()
				)
			);

		}
		else
		{
			// FALLING EDGE
			_sig = std::shared_ptr<rei::AbstractSignalInterface>(
				new SignalNoObstacleDetected(
					pubsubstate->msg_sub_replan_request_sig.header.stamp.toNSec()
				)
			);
		}
		planner_state_machine->propagateSignal(_sig);
	}

	virtual void executeUpdateVelocity()
	{
		current_speed = std::sqrt(
			pubsubstate->msg_sub_current_velocity.twist.linear.x*pubsubstate->msg_sub_current_velocity.twist.linear.x
			+ pubsubstate->msg_sub_current_velocity.twist.linear.y*pubsubstate->msg_sub_current_velocity.twist.linear.y
			+ pubsubstate->msg_sub_current_velocity.twist.linear.z*pubsubstate->msg_sub_current_velocity.twist.linear.z
		);
		trajectory_slicer.calcLookaheadDistance(pubsubstate->msg_sub_current_velocity,
			ref_velocity);
	}

	virtual bool initNode() override
	{
		using namespace teb_local_planner;
		conf.map_frame = "base_link";
		conf.robot.wheelbase = 2.7;
		conf.robot.min_turning_radius = 10.86;
		conf.optim.weight_kinematics_forward_drive = 10;
		conf.optim.weight_obstacle = 20.0;
		conf.obstacles.min_obstacle_dist = 0.2;
		conf.robot.max_vel_x_backwards = 0.0;
		conf.trajectory.dt_ref = 0.7;
		//conf.trajectory.max_samples = 100;
		conf.hcp.enable_multithreading = true;

		conf.trajectory.allow_init_with_backwards_motion = false;

		viz = TebVisualizationPtr(new TebVisualization(*nh, conf));
		Point2dContainer robot_footprint_points;
		Eigen::Vector2d p0(-0.15, -0.55);
		Eigen::Vector2d p1(-0.15, 0.55);
		Eigen::Vector2d p2(2.15, 0.55);
		Eigen::Vector2d p3(2.15, -0.55);
		robot_footprint_points.push_back(p0);
		robot_footprint_points.push_back(p1);
		robot_footprint_points.push_back(p2);
		robot_footprint_points.push_back(p3);
		robot_footprint = RobotFootprintModelPtr(new CircularRobotFootprint(2.0));
		obstacle_container = std::make_shared<ObstContainer>();
		// ROS
		planner = std::unique_ptr<teb_local_planner::TebOptimalPlanner>(
			new TebOptimalPlanner(
				conf,
				obstacle_container.get(),
				robot_footprint, viz,
				via_points.get()
			)
		);
		if (!initLocalPlannerStateMachine(nh, sync_state_machine))
		{
			return false;
		}
		timer_planner_sm = nh->createTimer(ros::Duration(1.0/(double)PLANNER_SYNC_HZ), &TebHotaruLocalPlanner::cycleSyncStateMachine, this);
		timer_planner_sm.start();
		sync_state_machine->setCbAllStateMessageReceived(
				std::bind(&TebHotaruLocalPlanner::startAfterAllMessagesReceived, this));
		return true;
	}

	void cycleSyncStateMachine(const ros::TimerEvent& event)
	{
		try
		{
			sync_state_machine->stepstatemachine();
		}
		catch(rei::StateMachineEmptySignalBuffer& e)
		{

		}
	}

	void cyclePlannerStateMachine(const ros::TimerEvent& event)
	{
		try
		{
			planner_state_machine->stepstatemachine();
		}
		catch(rei::StateMachineEmptySignalBuffer& e)
		{

		}
	}

	void startAfterAllMessagesReceived()
	{
		planner_state_machine->start();
		timer_planner_sm = nh->createTimer(ros::Duration(1.0/(double)PLANNER_SM_HZ), &TebHotaruLocalPlanner::cyclePlannerStateMachine, this);
		timer_planner_sm.start();
	}

	void executeUpdateClosestWaypoint()
	{
		trajectory_slicer.setOffset(pubsubstate->msg_closest_waypoint.data);
		// TODO: calculate waypoint velocity somehow
		ref_velocity = 5.0;
	}

	virtual void localPlanCycle() override
	{

		//
		if (starting_plan_points.size()>0)
		{
			//m_obstacle_update.lock();
			//conf.trajectory.max_samples = number_of_trajectory_points*2;
			try{
				std::cout << starting_plan_points.size() << '\n';
				plan_source_modification.lock();
				bool plan_success = planner->plan(starting_plan_points);
				plan_source_modification.unlock();
				if (plan_success)
				{
					pubsubstate->msg_final_waypoints.waypoints.clear();
					transformed_poses_current_pose.clear();
					planner->getFullTrajectory(_full_trajectory);
					if (pubsubstate->msg_sub_current_pose.pose.orientation.w != 0.0)
					{
						planner->visualize();
						pubsubstate->msg_final_waypoints.waypoints.clear();
						geometry_msgs::Pose _prev_pose;
						for (unsigned int i = 0; i < _full_trajectory.size(); i++)
						{
							geometry_msgs::Pose _pose_0;
							tf2::doTransform(
									_full_trajectory[i].pose,
									_pose_0,
									inv_transform_current_pose
							);
							if (rei::planarDistance(_prev_pose.position, _pose_0.position) > 0.4)
							{
								transformed_poses_current_pose.push_back(_pose_0);
								_prev_pose = _pose_0;
							}
						}
					}
					trajectory_slicer.joinWaypointsWithLocalPlan(
							pubsubstate->msg_sub_base_waypoints,
							transformed_poses_current_pose, current_speed,
							pubsubstate->msg_final_waypoints);
				}

			}catch(std::runtime_error& e){

			}
			//m_obstacle_update.unlock();
			//pub_final_waypoints.publish(final_waypoints);
		}

	}



	virtual void relayCycle() override
	{
		pubsubstate->msg_final_waypoints.waypoints.clear();
		if (pubsubstate->msg_closest_waypoint.data >= 0)
		{
			for (unsigned int i = pubsubstate->msg_closest_waypoint.data; i < pubsubstate->msg_sub_base_waypoints.waypoints.size(); i++)
			{
				autoware_msgs::Waypoint wp;
				wp = pubsubstate->msg_sub_base_waypoints.waypoints[i];
				//wp.twist.twist = v.velocity;
				//wp.twist = original_velocity_profile[i];
				pubsubstate->msg_final_waypoints.waypoints.push_back(
						std::move(wp));
			}
		}

	}

	void visualizeTeb()
	{

	}

	void maintThreadEvent(const ros::TimerEvent& e)
	{
		if (planner_state_machine->isReplanning())
		{
			localPlanCycle();
			visualizeTeb();
		}
		else if (planner_state_machine->isRelay())
		{
			relayCycle();
		}

		publishFinal_waypoints();
	}

	void mainThread()
	{
		timer_cycle = nh->createTimer(ros::Duration(1.0/((double)PLANNER_SM_HZ/2.0)), &TebHotaruLocalPlanner::maintThreadEvent, this);
		timer_cycle.start();
	}


};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "hotaru_teb_local_planner");
	std::shared_ptr<ros::NodeHandle> nh(new ros::NodeHandle());
	TebHotaruLocalPlanner teb_planner(nh);
	if (teb_planner.init())
	{
		ROS_INFO("Starting local planner component");
		teb_planner.mainThread();
		ros::AsyncSpinner async_spinner(4);
		async_spinner.start();
		ros::waitForShutdown();
		return 0;
	}
	else
	{
		ROS_ERROR("Unable to initialize local planner component");
		return 1;
	}
}
