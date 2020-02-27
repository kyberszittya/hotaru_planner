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





class TebHotaruLocalPlanner: public hotaru::InterfaceRos_Hotarulocalplanner,
	public hotaru::Abstract_RosLocalPlanner
{
private:
protected:
	std::shared_ptr<teb_local_planner::ObstContainer> obstacle_container;
	teb_local_planner::RobotFootprintModelPtr robot_footprint;
	teb_local_planner::TebConfig conf;
	teb_local_planner::TebVisualizationPtr viz;
	std::unique_ptr<teb_local_planner::TebOptimalPlanner> planner;
	std::shared_ptr<teb_local_planner::ViaPointContainer> via_points;
	// Starting plan as pose stamped
	std::vector<teb_local_planner::TrajectoryPointMsg> _full_trajectory;


	std::vector<geometry_msgs::Pose> transformed_poses_current_pose;



	virtual void executeReconstructWaypoints() override {
		trajectory_slicer.calcLookaheadIndex(pubsubstate->msg_sub_base_waypoints);
		reconstructStartingPlanPoints(pubsubstate->msg_sub_base_waypoints,
				pubsubstate->msg_sub_current_pose);
	}

	virtual void executePlannerMethods() override {

	}
public:
	TebHotaruLocalPlanner(std::shared_ptr<ros::NodeHandle> nh):
		InterfaceRos_Hotarulocalplanner(nh){}

	virtual void executeSynchWithPose() override
	{
		syncTfPose();
		//ROS_INFO("synchronize");
	}

	virtual void executeUpdateObstacles() override
	{
		//m_obstacle_update.lock();
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
		//m_obstacle_update.unlock();
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
		planner_state_machine->stepstatemachine();
	}

	virtual void executeUpdateVelocity()
	{
		trajectory_slicer.calcLookaheadDistance(pubsubstate->msg_sub_current_velocity);
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
		conf.trajectory.dt_ref = 0.5;
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
		sync_state_machine->setCbAllStateMessageReceived(
				std::bind(&TebHotaruLocalPlanner::startAfterAllMessagesReceived, this));
		return true;
	}

	void startAfterAllMessagesReceived()
	{
		planner_state_machine->start();
	}



	virtual void localPlanCycle() override
	{

		//
		if (starting_plan_points.size()>0)
		{
			//m_obstacle_update.lock();
			//conf.trajectory.max_samples = number_of_trajectory_points*2;
			if (planner->plan(starting_plan_points))
			{
				pubsubstate->msg_final_waypoints.waypoints.clear();
				transformed_poses_current_pose.clear();
				planner->getFullTrajectory(_full_trajectory);
				if (pubsubstate->msg_sub_current_pose.pose.orientation.w != 0.0)
				{
					planner->visualize();
					pubsubstate->msg_final_waypoints.waypoints.clear();
					for (unsigned int i = 0; i < _full_trajectory.size(); i++)
					{
						geometry_msgs::Pose _pose_0;
						tf2::doTransform(
								_full_trajectory[i].pose,
								_pose_0,
								inv_transform_current_pose
						);
						//autoware_msgs::Waypoint wp;
						//wp.pose.pose = _full_trajectory[i].pose;
						//wp.twist.twist = v.velocity;
						//wp.twist = original_velocity_profile[i];
						//wp.twist.twist.linear.x = 10/3.6;
						//pubsubstate->msg_final_waypoints.waypoints.push_back(
						//		std::move(wp));
						transformed_poses_current_pose.push_back(_pose_0);
					}
				}
				trajectory_slicer.joinWaypointsWithLocalPlan(
						pubsubstate->msg_sub_base_waypoints,
						transformed_poses_current_pose, pubsubstate->msg_sub_current_velocity,
						pubsubstate->msg_final_waypoints);
			}
			//m_obstacle_update.unlock();
			//pub_final_waypoints.publish(final_waypoints);
		}

	}

	virtual void relayCycle() override
	{

		pubsubstate->msg_final_waypoints.waypoints.clear();
		for (unsigned int i = 0; i < pubsubstate->msg_sub_base_waypoints.waypoints.size(); i++)
		{
			autoware_msgs::Waypoint wp;
			wp = pubsubstate->msg_sub_base_waypoints.waypoints[i];
			//wp.twist.twist = v.velocity;
			//wp.twist = original_velocity_profile[i];
			pubsubstate->msg_final_waypoints.waypoints.push_back(
					std::move(wp));
		}

	}

	void visualizeTeb()
	{


	}

	void mainThread()
	{
		if (planner_state_machine->isReplanning())
		{
			localPlanCycle();
			visualizeTeb();
		}
		else
		{
			relayCycle();
		}
		publishFinal_waypoints();
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
		ros::Rate r(40.0);
		while(ros::ok())
		{
			teb_planner.mainThread();
			ros::spinOnce();
			r.sleep();
		}
		return 0;
	}
	else
	{
		ROS_ERROR("Unable to initialize local planner component");
		return 1;
	}
}
