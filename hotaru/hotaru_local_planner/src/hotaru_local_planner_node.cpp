/*
 * hotaru_local_planner_node.cpp
 *
 *  Created on: Jan 13, 2020
 *      Author: kyberszittya
 */

#include <iostream>

#include <ros/ros.h>

#include <teb_local_planner/graph_search.h>
#include <teb_local_planner/teb_local_planner_ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <visualization_msgs/MarkerArray.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <hotaru_common/state_machine/state_machine.hpp>

#include <hotaru_common/planner_components/common_building_blocks.hpp>
#include <hotaru_common/planner_components/abstract_planner.hpp>

#include <visualization_msgs/MarkerArray.h>

#include <thread>
#include <mutex>

namespace hotaru {

class TebLocalPlanner: public Abstract_RosLocalPlanner
{
private:
protected:
	// ROS
	ros::Subscriber sub_obstacle_array;
	// Teb
	std::unique_ptr<teb_local_planner::TebOptimalPlanner> planner;
	std::shared_ptr<teb_local_planner::ViaPointContainer> via_points;
	std::shared_ptr<teb_local_planner::ObstContainer> obstacle_container;
	// Starting plan as pose stamped
	std::vector<teb_local_planner::TrajectoryPointMsg> _full_trajectory;

	teb_local_planner::TebVisualizationPtr viz;
	teb_local_planner::RobotFootprintModelPtr robot_footprint;
	teb_local_planner::TebConfig conf;
	// Mutex to obstacles
	std::mutex m_obstacle_update;
public:
	TebLocalPlanner(ros::NodeHandle& nh): Abstract_RosLocalPlanner(nh)
	{}

	virtual bool initNode() override
	{
		using namespace teb_local_planner;
		conf.map_frame = "map";
		conf.robot.wheelbase = 2.7;
		conf.robot.min_turning_radius = 10.86;
		conf.optim.weight_kinematics_forward_drive = 10;

		conf.optim.weight_obstacle = 200.0;
		conf.obstacles.min_obstacle_dist = 0.2;
		conf.robot.max_vel_x_backwards = 0.0;
		conf.trajectory.dt_ref = 0.5;
		conf.hcp.enable_homotopy_class_planning = true;
		conf.hcp.enable_multithreading = true;
		conf.trajectory.max_samples = 200;
		conf.trajectory.allow_init_with_backwards_motion = false;

		viz = TebVisualizationPtr(new TebVisualization(nh, conf));
		Point2dContainer robot_footprint_points;
		Eigen::Vector2d p0(-0.15, -0.55);
		Eigen::Vector2d p1(-0.15, 0.55);
		Eigen::Vector2d p2(2.15, 0.55);
		Eigen::Vector2d p3(2.15, -0.55);
		robot_footprint_points.push_back(p0);
		robot_footprint_points.push_back(p1);
		robot_footprint_points.push_back(p2);
		robot_footprint_points.push_back(p3);

		robot_footprint = RobotFootprintModelPtr(new TwoCirclesRobotFootprint(2.3, 1.05, -2.35, 1.05));
		obstacle_container = std::make_shared<ObstContainer>();
		// ROS
		sub_obstacle_array = nh.subscribe("/poly_detected_obstacles", 10, &TebLocalPlanner::subObstacleArray, this);

		planner = std::unique_ptr<teb_local_planner::TebOptimalPlanner>(
			new TebOptimalPlanner(
				conf,
				obstacle_container.get(),
				robot_footprint,
				viz,
				via_points.get()
				));
		return true;
	}

	void subObstacleArray(const visualization_msgs::MarkerArray::ConstPtr& msg)
	{
		m_obstacle_update.lock();
		obstacle_container->clear();
		using namespace teb_local_planner;
		for (auto& marker: msg->markers)
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
		m_obstacle_update.unlock();
	}

	void subGridMap(const grid_map_msgs::GridMap::ConstPtr& msg)
	{


	}

	virtual void localPlanCycle() override
	{

		if (starting_plan_points.size()>0)
		{
			m_obstacle_update.lock();
			if (planner->plan(starting_plan_points))
			{
				planner->getFullTrajectory(_full_trajectory);
				if (kinematic_state->pose.pose.orientation.w != 0.0)
				{
					planner->visualize();
					for (const auto& v: _full_trajectory)
					{
						autoware_msgs::Waypoint wp;
						wp.pose.pose = v.pose;
						wp.twist.twist = v.velocity;
						final_waypoints.waypoints.push_back(std::move(wp));
					}

				}
			}
			m_obstacle_update.unlock();
			pub_final_waypoints.publish(final_waypoints);
		}

	}

	void visualizeTeb()
	{


	}


};

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "hotaru_teb_local_planner");
	ros::NodeHandle nh;
	hotaru::TebLocalPlanner teb_planner(nh);
	if (teb_planner.init())
	{
		ROS_INFO("Starting local planner component");
		ros::Rate r(10.0);
		while(ros::ok())
		{
			teb_planner.localPlanCycle();
			teb_planner.visualizeTeb();
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
