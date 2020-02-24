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

class GuardLocalPlanner: public hotaru::Interface_GuardLocalPlanner
{
public:
	virtual bool guard_Relay2ReplanningState() override
	{
		return true;
	}
	virtual bool guard_Replanning2RelayState() override
	{
		return true;
	}
	virtual bool guard_Relay2Waiting() override
	{
		return true;
	}
	virtual bool guard_Waiting2Relay() override
	{
		return true;
	}
};

std::string translateTrajectorySignalToName(
		std::shared_ptr<rei::AbstractSignalInterface> _sig)
{
	switch(_sig->getId())
	{
		case 0x21: {
			return "SignalNoObstacleDetected";
		}
		case 0x22: {
			return "SignalLastWaypointReached";
		}
		case 0x23: {
			return "SignalNewGlobalPlan";
		}
		default: {
			return "";
		}
	}
}

class RosReplannerGraphNotifier: public rei::Interface_CommunicationGraphNotifier
{
private:
	std::string node_name;
	std::shared_ptr<ros::NodeHandle> nh;
	rei_monitoring_msgs::ReiStateMachineTransitionSignal msg_sig;
	ros::Publisher pub_sig_id;
public:
	RosReplannerGraphNotifier(std::string node_name,
			std::shared_ptr<ros::NodeHandle> nh):
				node_name(node_name), nh(nh) {}

	void initialize()
	{
		ROS_INFO_STREAM("Initializing ROS communication notifier: "
				<< node_name+"/sync_state_machine/current_state");
		pub_sig_id = nh->advertise<rei_monitoring_msgs::ReiStateMachineTransitionSignal>(
				node_name+"/replanner/current_state", 10);
	}


	virtual void notifyCommunicationGraph(std::shared_ptr<rei::AbstractSignalInterface> sig)
	{
		msg_sig.header.stamp = ros::Time::now();
		msg_sig.signal_name = translateTrajectorySignalToName(sig);
		msg_sig.sig_id = sig->getId();
		pub_sig_id.publish(msg_sig);
	}
};

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
	std::unique_ptr<hotaru::LocalPlannerStateMachine> planner_state_machine;
	// ROS communication graph as notifier of replanner state
	std::shared_ptr<RosReplannerGraphNotifier> comm_repl_notif;

	virtual void executeReconstructWaypoints() override {
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
		ROS_INFO("synchronize");
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

	virtual bool initNode() override
	{
		using namespace teb_local_planner;
		conf.map_frame = "map";
		conf.robot.wheelbase = 2.7;
		conf.robot.min_turning_radius = 10.86;
		conf.optim.weight_kinematics_forward_drive = 10;
		conf.optim.weight_viapoint = 50;
		conf.optim.weight_obstacle = 2000.0;
		conf.obstacles.min_obstacle_dist = 0.2;
		conf.robot.max_vel_x_backwards = 0.0;
		conf.trajectory.dt_ref = 0.5;
		conf.hcp.enable_multithreading = true;
		conf.trajectory.max_samples = 40;
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
				robot_footprint,
				viz,
				via_points.get()
				));
		// Initialize planner state machine
		comm_repl_notif = std::shared_ptr<RosReplannerGraphNotifier>(
			new rei::RosCommunicationGraphNotifier("local_planner_state", nh)
		);
		std::unique_ptr<GuardLocalPlanner> guard_local_planner =
				std::unique_ptr<GuardLocalPlanner>(new GuardLocalPlanner());
		planner_state_machine = std::unique_ptr<hotaru::LocalPlannerStateMachine>(
				new hotaru::LocalPlannerStateMachine(comm_repl_notif,
						std::move(guard_local_planner)));
		planner_state_machine->start();
		return true;
	}

	virtual void localPlanCycle() override
	{
		if (starting_plan_points.size()>0)
		{
			//m_obstacle_update.lock();
			conf.trajectory.max_samples = number_of_trajectory_points;
			if (planner->plan(starting_plan_points))
			{
				planner->getFullTrajectory(_full_trajectory);
				if (pubsubstate->msg_sub_current_pose.pose.orientation.w != 0.0)
				{
					planner->visualize();
					pubsubstate->msg_final_waypoints.waypoints.clear();
					for (unsigned int i = 0; i < _full_trajectory.size(); i++)
					{
						autoware_msgs::Waypoint wp;
						wp.pose.pose = _full_trajectory[i].pose;
						//wp.twist.twist = v.velocity;
						//wp.twist = original_velocity_profile[i];
						wp.twist.twist.linear.x = 10/3.6;
						pubsubstate->msg_final_waypoints.waypoints.push_back(
								std::move(wp));
					}

				}
			}
			//m_obstacle_update.unlock();
			//pub_final_waypoints.publish(final_waypoints);
		}

	}

	void visualizeTeb()
	{


	}

	void mainThread()
	{
		localPlanCycle();
		visualizeTeb();
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
		ros::Rate r(10.0);
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
