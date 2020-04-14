/*
 * obstacle_detector_node.cpp
 *
 *  Created on: Feb 12, 2020
 *      Author: kyberszittya
 */

#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point32.h>

#include <std_msgs/Int32.h>

#include <visualization_msgs/MarkerArray.h>

#include <rei_planner_signals/ReplanRequest.h>
#include <eigen_conversions/eigen_msg.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <rei_common/geometric_utilities.hpp>

#include <autoware_msgs/DetectedObjectArray.h>

#include <hotaru_msgs/RefinedTrajectory.h>
#include <rei_monitoring_msgs/DetectedObstacles.h>
#include <rei_common/geometric_utilities.hpp>

constexpr double D_LATERAL_THRESHOLD = 2.75;
constexpr double D_LONGITUDINAL_THRESHOLD = 5.75;

namespace rei
{

class ObstacleGridMapMonitor
{
private:
	int closest_waypoint;
protected:

	geometry_msgs::PoseStamped current_pose;
	ros::NodeHandle nh;
	ros::Subscriber sub_grid_map;
	ros::Subscriber sub_current_pose;
	ros::Subscriber sub_base_waypoints;
	ros::Subscriber sub_closest_waypoint;
	ros::Subscriber sub_obstacle_poly;
	ros::Publisher pub_replan_signal;
	ros::Publisher pub_local_gridmap;
	rei_monitoring_msgs::DetectedObstacles detected_obstacles;
	ros::Publisher pub_detected_obstacles;			 				///< Detected obstacles
	grid_map::GridMap global_map;
	rei_planner_signals::ReplanRequest request_msg;
	hotaru_msgs::RefinedTrajectory msg_base_waypoints;
	// Detect polygon obstacles
	visualization_msgs::MarkerArray input_obstacles;
	autoware_msgs::DetectedObjectArray poly_obstacle_array;
	visualization_msgs::MarkerArray obstacles;
	tf2_ros::Buffer tf_buffer;
	std::unique_ptr<tf2_ros::TransformListener> tf_listener;
	geometry_msgs::TransformStamped transform_current_pose;
	geometry_msgs::TransformStamped inv_transform_current_pose;
	geometry_msgs::TransformStamped transform_detector;
public:
	ObstacleGridMapMonitor(ros::NodeHandle& nh): closest_waypoint(-1), nh(nh){}

	bool init()
	{
		tf_listener = std::unique_ptr<tf2_ros::TransformListener>(
			new tf2_ros::TransformListener(tf_buffer)
		);
		pub_detected_obstacles = nh.advertise<visualization_msgs::MarkerArray>("/viz_filtered_obstacles", 10);
		pub_replan_signal = nh.advertise<rei_planner_signals::ReplanRequest>("/replan_request_sig",10);
		sub_current_pose = nh.subscribe("current_pose", 10, &ObstacleGridMapMonitor::cbCurrentPose, this);
		sub_closest_waypoint = nh.subscribe("/closest_waypoint", 10, &ObstacleGridMapMonitor::cbClosestWaypoint, this);
		sub_grid_map = nh.subscribe("grid_map", 1, &ObstacleGridMapMonitor::subGridMap, this);
		sub_base_waypoints = nh.subscribe("input_trajectory", 1, &ObstacleGridMapMonitor::cbBaseWaypoints, this);
		// Sub polygon obstacles
		//pub_detected_obstacles_array = nh.advertise<autoware_msgs::DetectedObjectArray>("/filtered_obstacles_poly", 10);
		pub_local_gridmap = nh.advertise<grid_map_msgs::GridMap>("/local_grid_map", 1);
		pub_detected_obstacles = nh.advertise<rei_monitoring_msgs::DetectedObstacles>("/rei_perception_monitor/detected_obstacles", 1);
		sub_obstacle_poly = nh.subscribe("/detection/lidar_detector/objects", 1, &ObstacleGridMapMonitor::cbPolyObj, this);


		return true;
	}

	void cbPolyObj(const autoware_msgs::DetectedObjectArray::ConstPtr& msg)
	{
		poly_obstacle_array.objects.clear();
		// Filter obstacles of interest
		double longitudinal_distance = 0.0;
		if (closest_waypoint >= 1)
		{
			detected_obstacles.obstacles.clear();
			for (unsigned int i = closest_waypoint; i < msg_base_waypoints.waypoints.size(); i++)
			{
				geometry_msgs::PoseStamped _pose_0;
				tf2::doTransform(
						msg_base_waypoints.waypoints[i-1].pose,
						_pose_0,
						transform_current_pose
				);
				geometry_msgs::PoseStamped _pose_1;
				tf2::doTransform(
						msg_base_waypoints.waypoints[i].pose,
						_pose_1,
						transform_current_pose
				);
				if (rei::isInvalidPoint(_pose_1.pose.position)){ break; }
				longitudinal_distance += planarDistance(
					_pose_0.pose.position,
					_pose_1.pose.position
				);
				if (longitudinal_distance >= 150.0) { break; }
				if (longitudinal_distance >= 6.0)
				{
					// Typical application of polytopes!
					// QHULL or CCD should be included or FCL
					for (const auto& o: msg->objects)
					{
						if (o.pose.position.x > 1.0)
						{
							double d_lateral = distanceToLine(_pose_0.pose.position,
									_pose_1.pose.position, o.pose.position);
							if (d_lateral <= D_LATERAL_THRESHOLD)
							{
								///ROS_INFO("Obstacle detected");
								rei_monitoring_msgs::Obstacle o1;
								o1.obstacle_type = rei_monitoring_msgs::Obstacle::STATIC_OBSTACLE;
								o1.radius = 1.2;
								tf2::doTransform(o.pose, o1.pose, transform_detector);
								detected_obstacles.obstacles.push_back(std::move(o1));
								request_msg.obstacle_min_lateral_distance = d_lateral;
								request_msg.obstacle_min_longitudinal_distance = longitudinal_distance;
								request_msg.eval = true;
								break;
							}
						}
					}
				}
			}
			if (poly_obstacle_array.objects.size()==0)
			{
				request_msg.eval = false;
			}
		}
		// Publish replan request
		request_msg.header.stamp = ros::Time::now();
		pub_replan_signal.publish(request_msg);
		pub_detected_obstacles.publish(detected_obstacles);
	}

	void cbBaseWaypoints(const hotaru_msgs::RefinedTrajectory::ConstPtr& msg)
	{
		msg_base_waypoints = *msg;
	}

	void cbClosestWaypoint(const std_msgs::Int32::ConstPtr& msg)
	{
		closest_waypoint = msg->data;
	}

	void cbCurrentPose(const geometry_msgs::PoseStamped::ConstPtr& msg)
	{
		current_pose = *msg;
		try {
			transform_current_pose = tf_buffer.lookupTransform(
					"base_link", "map", ros::Time(0), ros::Duration(1.0));
			transform_detector = tf_buffer.lookupTransform(
					"ouster_left", "base_link", ros::Time(0), ros::Duration(1.0)
			);
		}catch(tf2::LookupException& le)
		{
			ROS_ERROR_STREAM(le.what());
		}catch(tf2::ExtrapolationException& ee)
		{
			ROS_ERROR_STREAM(ee.what());
		}catch(tf2::ConnectivityException& e)
		{
			ROS_ERROR_STREAM(e.what());
		}

	}

	void cbObstacleCluster(const visualization_msgs::MarkerArray::ConstPtr& msg)
	{
		obstacles.markers.clear();
		// TODO: this part is pretty nasty, check it
		input_obstacles = *msg;
		filterObstacles();
		pub_detected_obstacles.publish(obstacles);
		// Publish replan request
		request_msg.header.stamp = ros::Time::now();
		pub_replan_signal.publish(request_msg);
	}

	void filterObstacles()
	{
		double longitudinal_distance = 0.0;

		if (closest_waypoint >= 1)
		{
			for (unsigned int i = closest_waypoint; i < msg_base_waypoints.waypoints.size(); i++)
			{
				geometry_msgs::PoseStamped _pose_0;
				tf2::doTransform(
						msg_base_waypoints.waypoints[i-1].pose,
						_pose_0,
						transform_current_pose
				);
				geometry_msgs::PoseStamped _pose_1;
				tf2::doTransform(
						msg_base_waypoints.waypoints[i].pose,
						_pose_1,
						transform_current_pose
				);
				longitudinal_distance += planarDistance(
					_pose_0.pose.position,
					_pose_1.pose.position
				);
				if (longitudinal_distance > 150){  break; }
				for (const auto& m: input_obstacles.markers)
				{
					if (planarDistance(
							_pose_1.pose.position, m.pose.position) < 4.0
							&& longitudinal_distance >= 6.0)
					{
						// Check lateral distance
						double d_lateral = distanceToLine(_pose_0.pose.position,
								_pose_1.pose.position, m.pose.position);
						d_lateral = sqrt(d_lateral*d_lateral);
						if (d_lateral <= D_LATERAL_THRESHOLD)
						{
							///ROS_INFO("Obstacle detected");
							if (request_msg.obstacle_min_lateral_distance > d_lateral)
							{
								request_msg.obstacle_min_lateral_distance = d_lateral;
								request_msg.obstacle_min_longitudinal_distance = longitudinal_distance;
							}
							request_msg.eval = true;
							obstacles.markers.push_back(m);
						}

					}
				}

			}
			if (obstacles.markers.size()==0)
			{
				request_msg.eval = false;
			}
		}
		else
		{
			request_msg.eval = false;
		}

	}

	void subGridMap(const grid_map_msgs::GridMap::ConstPtr& msg)
	{
		grid_map::GridMapRosConverter::fromMessage(*msg, global_map);
		grid_map::Position pos_current(
						current_pose.pose.position.x,
						current_pose.pose.position.y);
		obstacles.markers.clear();
		auto t = ros::Time::now();
		request_msg.obstacle_min_longitudinal_distance = std::numeric_limits<double>::max();
		request_msg.obstacle_min_lateral_distance = std::numeric_limits<double>::max();
		request_msg.eval = false;
		for (grid_map::GridMapIterator iterator(global_map);
							!iterator.isPastEnd(); ++iterator)
		{

			if (global_map.at("elevation", *iterator) > 0.5)
			{
				visualization_msgs::Marker m;
				m.header.frame_id = "base_link";
				m.header.stamp = t;
				m.type = visualization_msgs::Marker::SPHERE;
				grid_map::Position3 pos;
				global_map.getPosition3("elevation", *(iterator), pos);
				m.pose.position.x = pos[0];
				m.pose.position.y = pos[1];
				m.pose.orientation.w = 1.0;
				m.scale.x = global_map.getResolution()*2.0;
				m.scale.y = global_map.getResolution()*2.0;
				m.scale.z = global_map.getResolution()*2.0;
				// Set color
				m.color.a = 1.0;
				m.color.r = 0.13;
				m.color.g = 0.7;
				m.color.b = 0.8;
				obstacles.markers.push_back(std::move(m));
				// Check distance from base waypoints

			}

		}
		filterObstacles();
		pub_detected_obstacles.publish(obstacles);
		// Publish replan request
		request_msg.header.stamp = ros::Time::now();
		pub_replan_signal.publish(request_msg);
	}
};

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "rei_obstacle_monitor");
	ros::NodeHandle nh;
	rei::ObstacleGridMapMonitor obstacle_monitor(nh);
	if (obstacle_monitor.init())
	{
		ROS_INFO("Successfully started obstacle detection");
		ros::spin();
		return 0;
	}
	return -1;
}
