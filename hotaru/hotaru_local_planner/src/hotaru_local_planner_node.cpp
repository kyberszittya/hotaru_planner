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

#include <rei_monitoring_msgs/ReiStateTransition.h>
#include <std_msgs/Float64.h>
#include <autoware_msgs/VehicleStatus.h>


constexpr unsigned int PLANNER_SM_HZ = { 40 };
constexpr unsigned int PLANNER_SYNC_HZ = { 100 };



class TebHotaruLocalPlanner: public hotaru::InterfaceRos_Hotarulocalplanner,
	public hotaru::Abstract_RosLocalPlanner
{
private:
	double current_speed;
	double ref_velocity;
protected:
	// Obstacle handling
	std::map<unsigned int, teb_local_planner::ObstaclePtr> semantic_perimeter;
	std::vector<teb_local_planner::ObstaclePtr> lookahead_obstacles_instances;
	//
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
	ros::Timer timer_obstacle_update;
	// State transition
	ros::Publisher pub_replanner_sm_state;

	std::mutex plan_source_modification;
	std::vector<geometry_msgs::Pose> transformed_poses_current_pose;
	// Quantitative information
	ros::Publisher pub_plan_time;
	ros::Subscriber sub_vehicle_status;
	ros::Publisher pub_marker_end_index;
	ros::Publisher pub_marker_velocity_index;
	ros::Subscriber sub_semantic_obstacles;
	ros::Subscriber sub_semantic_road_line;
	int relay_minimal_obstacle_index;
	int minimalObstacleIndex;
	int horizon_length;
	visualization_msgs::Marker marker_obstacle_end_index;
	visualization_msgs::Marker marker_obstacle_velocity_index;



	virtual void executeReconstructWaypoints() override {
		plan_source_modification.lock();
		trajectory_slicer.calcLookaheadIndex(pubsubstate->msg_sub_base_waypoints);
		if (planner_state_machine->isRelay())
		{
			reconstructStartingPlanPoints(
				pubsubstate->msg_sub_base_waypoints,
				pubsubstate->msg_sub_current_pose,
				pubsubstate->msg_closest_waypoint.data
			);
		}
		else if (planner_state_machine->isReplanning())
		{
			reconstructStartingPlanPoints(
				pubsubstate->msg_sub_base_waypoints,
				pubsubstate->msg_sub_current_pose,
				pubsubstate->msg_closest_waypoint.data, 10
			);
		}
		// Unlock source modification
		plan_source_modification.unlock();
	}

	virtual void executePlannerMethods() override {

	}
public:
	TebHotaruLocalPlanner(std::shared_ptr<ros::NodeHandle> private_nh,
			std::shared_ptr<ros::NodeHandle> nh):
		ref_velocity(0.0),
		current_speed(0.0),
		horizon_length(0),
		minimalObstacleIndex(0),
		relay_minimal_obstacle_index(0),
		InterfaceRos_Hotarulocalplanner(private_nh, nh){}

	virtual void executeSynchWithPose() override
	{
		plan_source_modification.lock();
		syncTfPose();
		guard_local_planner->setCurrentPoint(pubsubstate->msg_sub_current_pose.pose.position);
		guard_local_planner->setCurrentLookaheadIndex(pubsubstate->msg_closest_waypoint.data);
		plan_source_modification.unlock();
	}

	virtual void executeUpdateObstacles() override
	{
		// Store current index
		int current_wp_index = pubsubstate->msg_closest_waypoint.data;
		//

		if (planner_state_machine->isRelay() ||
				pubsubstate->msg_sub_filtered_obstacles.objects.size() > 0)
		{
			//obstacle_container->clear();
			//poly_objects.clear();
			plan_source_modification.lock();
			lookahead_obstacles_instances.clear();
			plan_source_modification.unlock();
		}

		using namespace teb_local_planner;

		/*
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
		}*/

		if (lookahead_obstacles_instances.size() != pubsubstate->msg_sub_filtered_obstacles.objects.size())
		{
			plan_source_modification.lock();
			lookahead_obstacles_instances.clear();
			lookahead_obstacles_instances.reserve(pubsubstate->msg_sub_filtered_obstacles.objects.size());
			for (int i = 0; i < pubsubstate->msg_sub_filtered_obstacles.objects.size(); i++)
			{
				lookahead_obstacles_instances.push_back();
			}
			plan_source_modification.unlock();
		}
		int objects = 0;
		for (const auto &o: pubsubstate->msg_sub_filtered_obstacles.objects)
		{
			Point2dContainer points;
			points.reserve(o.convex_hull.polygon.points.size());
			for (const auto& p: o.convex_hull.polygon.points)
			{
				Eigen::Vector2d p1(p.x, p.y);
				points.push_back(p1);
			}

			if (planner_state_machine->isReplanning())
			{
				// Set new obstacle index
				int raw_obstacle_index = current_wp_index + calcObstacleMinimalIndex(
					o.pose.position,
					pubsubstate->msg_sub_base_waypoints.waypoints.size()
				);
				int new_index = current_wp_index + calcObstacleMinimalIndex(
					o.pose.position,
					pubsubstate->msg_sub_base_waypoints.waypoints.size(),
					2.5
				);
				if (new_index > relay_minimal_obstacle_index)
				{
					relay_minimal_obstacle_index = new_index;
				}
				// All in all: TEB implementation acts strange on obstacle handling.
				// That should be reviewed. Also, why not utilize FCL or any other library
				// for computing distances? That would be cool
				// Push into the tree
				plan_source_modification.lock();

				if (lookahead_obstacles_instances.push_back(objects)!=lookahead_obstacles_instances.end())
				{
					auto p = dynamic_cast<PolygonObstacle*>(lookahead_obstacles_instances[objects].get());
					p->clearVertices();
					p->vertices().reserve(points.size());
					p->vertices().assign(points.begin(), points.end());
				}
				else
				{
					lookahead_obstacles_instances.insert(
						std::pair<unsigned int, ObstaclePtr>(objects, new PolygonObstacle(points))
					);
				}
				plan_source_modification.unlock();
				// Calculate minimal velocity distance index
				marker_obstacle_end_index.pose.position =
						pubsubstate->msg_sub_base_waypoints
							.waypoints[relay_minimal_obstacle_index].pose.pose.position;
				marker_obstacle_end_index.header.frame_id = pubsubstate->msg_sub_base_waypoints.header.frame_id;
				guard_local_planner->setLookaheadPoint(
					pubsubstate->msg_sub_base_waypoints
						.waypoints[trajectory_slicer.getLookaheadIndex()]
						.pose.pose.position,
					minimalObstacleIndex,
					relay_minimal_obstacle_index,
					pubsubstate->msg_sub_base_waypoints.waypoints.size()
				);
				pub_marker_end_index.publish(marker_obstacle_end_index);
				pub_marker_velocity_index.publish(marker_obstacle_velocity_index);
			}
			objects++;
		}


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
		current_speed = std::sqrt(
			pubsubstate->msg_sub_current_velocity.twist.linear.x*pubsubstate->msg_sub_current_velocity.twist.linear.x
			+ pubsubstate->msg_sub_current_velocity.twist.linear.y*pubsubstate->msg_sub_current_velocity.twist.linear.y
			+ pubsubstate->msg_sub_current_velocity.twist.linear.z*pubsubstate->msg_sub_current_velocity.twist.linear.z
		);
		trajectory_slicer.calcLookaheadDistance(pubsubstate->msg_sub_current_velocity,
			ref_velocity,
			horizon_length);
	}

	virtual bool initNode() override
	{
		// Visualization
		pub_marker_end_index = nh->advertise<visualization_msgs::Marker>(
			"/hotaru_local_planner/obstacle_index",1);
		pub_marker_velocity_index = nh->advertise<visualization_msgs::Marker>(
			"/hotaru_local_planner/velocity_index",1);
		marker_obstacle_end_index.color.r = 0.0;
		marker_obstacle_end_index.color.g = 1.0;
		marker_obstacle_end_index.color.b = 1.0;
		marker_obstacle_end_index.color.a = 1.0;
		marker_obstacle_end_index.scale.x = 1.0;
		marker_obstacle_end_index.scale.y = 1.0;
		marker_obstacle_end_index.scale.z = 1.0;
		marker_obstacle_end_index.pose.orientation.w = 1.0;
		marker_obstacle_end_index.type = visualization_msgs::Marker::SPHERE;
		marker_obstacle_end_index.text = "OBSTACLE_END";
		marker_obstacle_velocity_index.color.r = 1.0;
		marker_obstacle_velocity_index.color.g = 0.0;
		marker_obstacle_velocity_index.color.b = 0.0;
		marker_obstacle_velocity_index.color.a = 1.0;
		marker_obstacle_velocity_index.scale.x = 1.0;
		marker_obstacle_velocity_index.scale.y = 1.0;
		marker_obstacle_velocity_index.scale.z = 1.0;
		marker_obstacle_velocity_index.pose.orientation.w = 1.0;
		marker_obstacle_velocity_index.type = visualization_msgs::Marker::SPHERE;
		marker_obstacle_velocity_index.text = "OBSTACLE_END";
		//
		sub_vehicle_status = nh->subscribe("/vehicle_status", 10, &TebHotaruLocalPlanner::cbVehicleStatue, this);
		// Benchmark
		pub_plan_time = nh->advertise<std_msgs::Float64>("/hotaru_local_planner/plan_time", 10);
		// State machine
		pub_replanner_sm_state = nh->advertise<rei_monitoring_msgs::ReiStateTransition>(
				"/hotaru_local_planner/state_transition", 10);
		// Configure teb
		using namespace teb_local_planner;
		conf.map_frame = "base_link";
		// Set parameters from private node handle
		// Wheelbase parameter
		double wheelbase = 2.7;
		if (!private_nh->getParam("wheelbase", wheelbase))
		{
			ROS_WARN_STREAM("Using default wheelbase: " << wheelbase);
		}
		conf.robot.wheelbase = wheelbase;
		// Minimal turning radius
		double min_turning_radius = 10.86;
		if (!private_nh->getParam("min_turning_radius", min_turning_radius))
		{
			ROS_WARN_STREAM("Using default minimum turning radius: " << min_turning_radius);
		}
		conf.robot.min_turning_radius = min_turning_radius;
		// Kinematics forward drive
		double weight_kinematics_forward_drive = 100;
		if (!private_nh->getParam("weight_kinematics_forward_drive", weight_kinematics_forward_drive))
		{
			ROS_WARN_STREAM("Using default weight kinematics forward drive: " << weight_kinematics_forward_drive);
		}
		conf.optim.weight_kinematics_forward_drive = weight_kinematics_forward_drive;
		// Obstacle weight
		double weight_obstacle = 95;
		if (!private_nh->getParam("weight_obstacle", weight_obstacle))
		{
			ROS_WARN_STREAM("Using default weight kinematics forward drive: " << weight_obstacle);
		}
		conf.optim.weight_obstacle = weight_obstacle;
		// Minimal distance
		double min_obstacle_distance = 1.2;
		if (!private_nh->getParam("min_obstacle_distance", min_obstacle_distance))
		{
			ROS_WARN_STREAM("Using min obstacle distance " << min_obstacle_distance);
		}
		conf.obstacles.min_obstacle_dist = min_obstacle_distance;
		// Acc lim theta
		double weight_theta = 0.1;
		conf.optim.weight_acc_lim_theta = weight_theta;
		// Weight adapt factor
		double weight_adapt_factor = 2.5;
		if (!private_nh->getParam("weight_adapt_factor", weight_adapt_factor))
		{
			ROS_WARN_STREAM("Using weight adapt factor " << weight_adapt_factor);
		}
		//conf.optim.weight_adapt_factor = weight_adapt_factor;
		// Obstacle cost exponent
		//double obstacle_cost_exponent = 1.15;
		//if (!private_nh->getParam("obstacle_cost_exponent", obstacle_cost_exponent))
		//{
		//	ROS_WARN_STREAM("Using obstacle cost exponent " << obstacle_cost_exponent);
		//}
		//conf.optim.obstacle_cost_exponent = obstacle_cost_exponent;
		// Config max vel backwards
		conf.robot.max_vel_x_backwards = 0.0;
		conf.trajectory.dt_ref = 1.5;
		//conf.optim.weight_max_vel_x = 10;
		//conf.trajectory.max_samples = 250;
		// No omni-directional motion
		conf.optim.weight_kinematics_nh = 2000; /// This must be kurva nagy
		conf.optim.weight_acc_lim_x = 0.5;
		conf.optim.weight_acc_lim_y = 0.0;
		conf.optim.weight_max_vel_y = 0.0;
		conf.optim.weight_viapoint = 1.0;
		conf.optim.penalty_epsilon = 0.6;
		conf.optim.weight_prefer_rotdir = 100;
		conf.goal_tolerance.xy_goal_tolerance = 1.0;
		conf.goal_tolerance.yaw_goal_tolerance = 1.0;
		//conf.optim.weight_kinematics_turning_radius = 20.0;
		//conf.optim.weight_shortest_path = 10;
		conf.hcp.enable_multithreading = true;
		conf.hcp.simple_exploration = true;
		//conf.hcp.delete_detours_backwards = true;
		conf.trajectory.allow_init_with_backwards_motion = false;
		conf.trajectory.feasibility_check_no_poses = 7;

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
		robot_footprint = RobotFootprintModelPtr(
				new TwoCirclesRobotFootprint(2.0, 1.5, 2.0, 1.5));
		//robot_footprint = RobotFootprintModelPtr(new CircularRobotFootprint(2.0));
		obstacle_container = std::make_shared<ObstContainer>();
		via_points = std::make_shared<ViaPointContainer>();
		// ROS
		planner = std::unique_ptr<teb_local_planner::TebOptimalPlanner>(
			new TebOptimalPlanner(
				conf,
				obstacle_container.get(),
				robot_footprint, viz,
				via_points.get()
			)
		);
		// Initialize sync state machine
		if (!initLocalPlannerStateMachine(nh, sync_state_machine))
		{
			return false;
		}
		// Set callbacks for replanning state machine
		planner_state_machine->setCbRelay(
			std::bind(&TebHotaruLocalPlanner::actRelayState, this)
		);
		planner_state_machine->setCbWaiting(
			std::bind(&TebHotaruLocalPlanner::actWaitingState, this)
		);
		planner_state_machine->setCbReplanningEnter(
			std::bind(&TebHotaruLocalPlanner::actReplanningState, this)
		);
		// Start planner SM
		timer_planner_sm = nh->createTimer(ros::Duration(1.0/(double)PLANNER_SYNC_HZ), &TebHotaruLocalPlanner::cycleSyncStateMachine, this);
		timer_planner_sm.start();
		sync_state_machine->setCbAllStateMessageReceived(
			std::bind(&TebHotaruLocalPlanner::startAfterAllMessagesReceived, this)
		);
		// Semantic line subscriber
		sub_semantic_road_line = nh->subscribe("/map_semantic_line", 1,
			&TebHotaruLocalPlanner::cbSemanticLineObstacles, this
		);
		// Start obstacle update
		timer_obstacle_update = nh->createTimer(
			ros::Duration(1.0/(double)PLANNER_SYNC_HZ),
			&TebHotaruLocalPlanner::cycleObstacleUpdate, this);
		timer_obstacle_update.start();
		// Return everything is OK
		return true;
	}

	void cycleObstacleUpdate(const ros::TimerEvent& event)
	{
		plan_source_modification.lock();
		obstacle_container->clear();
		for (const auto& o: lookahead_obstacles_instances)
		{
			obstacle_container->push_back(o.second);
		}
		for (const auto& o: semantic_perimeter)
		{
			obstacle_container->push_back(o.second);
		}
		plan_source_modification.unlock();
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

	void actRelayState()
	{
		rei_monitoring_msgs::ReiStateTransition sig;
		sig.header.stamp = ros::Time::now();
		sig.transition_signal = "RELAY";
		pub_replanner_sm_state.publish(sig);
		plan_source_modification.lock();
		planner->clearPlanner();
		plan_source_modification.unlock();
		obstacle_container->clear();
		this->relay_minimal_obstacle_index = 0;
	}

	void actReplanningState()
	{
		// Current waypoint index
		int current_index = pubsubstate->msg_closest_waypoint.data;
		//
		rei_monitoring_msgs::ReiStateTransition sig;
		sig.header.stamp = ros::Time::now();
		sig.transition_signal = "REPLANNING";
		pub_replanner_sm_state.publish(sig);
		geometry_msgs::Point _p0;
		tf2::doTransform(
			pubsubstate->msg_sub_current_pose.pose.position,
			_p0, transform_current_pose
		);
		minimalObstacleIndex = calcObstacleMinimalIndex(
			_p0, pubsubstate->msg_sub_base_waypoints.waypoints.size()
			) + current_index;
		// Horizon calculation
		/*horizon_length = calcObstacleMinimalIndex(
			_p0, pubsubstate->msg_sub_base_waypoints.waypoints.size(),
			1.5, 5
		);*/
		horizon_length = 20;
		marker_obstacle_velocity_index.pose.position =
		pubsubstate->msg_sub_base_waypoints.waypoints[minimalObstacleIndex].pose.pose.position;
		marker_obstacle_velocity_index.header.frame_id = pubsubstate->msg_sub_base_waypoints.header.frame_id;
	}

	void actWaitingState()
	{
		rei_monitoring_msgs::ReiStateTransition sig;
		sig.header.stamp = ros::Time::now();
		sig.transition_signal = "WAITING";
		pub_replanner_sm_state.publish(sig);
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

		if (starting_plan_points.size()>0)
		{
			try{
				auto t_start = std::chrono::steady_clock::now();
				plan_source_modification.lock();
				bool plan_success = planner->plan(starting_plan_points,
						&(pubsubstate->msg_sub_current_velocity.twist), true);
				plan_source_modification.unlock();
				auto t_end = std::chrono::steady_clock::now();
				std::chrono::duration<double> elps = t_end-t_start;
				std::cout << elps.count() << std::endl;
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

			}catch(std::out_of_range& e){
				ROS_ERROR("ERROR");
			}
		}

	}

	int calcObstacleMinimalIndex(const geometry_msgs::Point& ref_position,
			int max_length, const double scale = 1.0, const double minimal_distance = 0.0)
	{
		if (std::isnan(ref_position.x)) return 0;
		double speed = sqrt(
			pubsubstate->msg_sub_current_velocity.twist.linear.x*
			pubsubstate->msg_sub_current_velocity.twist.linear.x+
			pubsubstate->msg_sub_current_velocity.twist.linear.y*
			pubsubstate->msg_sub_current_velocity.twist.linear.y
		);
		double longitude_distance = 0.0;
		int dindex = 0;
		int obstacle_index = -1;
		int start_index = pubsubstate->msg_closest_waypoint.data;
		double threshold_distance = std::max(speed*scale, minimal_distance);
		for (int i = start_index+1;
				i < pubsubstate->msg_sub_base_waypoints.waypoints.size(); i++)
		{
			geometry_msgs::Point _p0;
			geometry_msgs::Point _p1;
			tf2::doTransform(
				pubsubstate->msg_sub_base_waypoints.waypoints[i-1].pose.pose.position,
				_p0, transform_current_pose
			);
			tf2::doTransform(
				pubsubstate->msg_sub_base_waypoints.waypoints[i].pose.pose.position,
				_p1, transform_current_pose
			);
			//double d = rei::planarDistance(ref_position, _p0);
			double d = ref_position.x - _p0.x;
			if (d < 1.0)
			{
				obstacle_index = i - start_index;
			}
			if (obstacle_index > -1)
			{
				double d = rei::planarDistance(
					_p0, _p1
				);
				longitude_distance += d;
				if (longitude_distance > threshold_distance)
				{
					return std::min(
						max_length,
						static_cast<int>(obstacle_index + dindex)
					);
				}
				dindex++;
			}
		}
		return 0;
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

	void cbVehicleStatue(const autoware_msgs::VehicleStatus::ConstPtr& msg)
	{

	}

	void cbSemanticLineObstacles(const visualization_msgs::MarkerArray::ConstPtr& msg)
	{
		using namespace teb_local_planner;
		for (const auto m: msg->markers)
		{
			plan_source_modification.lock();
			if (m.points.size()==2)
			{
				geometry_msgs::Point _lp0;
				geometry_msgs::Point _lp1;
				tf2::doTransform(m.points[0], _lp0, transform_current_pose);
				tf2::doTransform(m.points[1], _lp1, transform_current_pose);
				const Eigen::Vector2d l0(_lp0.x, _lp0.y);
				const Eigen::Vector2d l1(_lp1.x, _lp1.y);
				if (semantic_perimeter.find(m.id)!=semantic_perimeter.end())
				{
					auto p = dynamic_cast<LineObstacle*>(semantic_perimeter[m.id].get());
					p->setStart(l0);
					p->setEnd(l1);
				}
				else
				{
					semantic_perimeter.insert(std::pair<unsigned int, ObstaclePtr>(m.id,
						ObstaclePtr(
							new LineObstacle(
								_lp0.x, _lp0.y,
								_lp1.x, _lp1.y)
							)
						)
					);
				}
			}
			plan_source_modification.unlock();
		}
	}

	void visualizeTeb()
	{

	}

	void maintThreadEvent(const ros::TimerEvent& e)
	{
		std_msgs::Float64 plan_time;
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
		plan_time.data = e.current_expired.toSec() - e.last_expired.toSec();
		pub_plan_time.publish(plan_time);
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
	std::shared_ptr<ros::NodeHandle> private_nh(new ros::NodeHandle("~"));
	TebHotaruLocalPlanner teb_planner(private_nh, nh);
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
