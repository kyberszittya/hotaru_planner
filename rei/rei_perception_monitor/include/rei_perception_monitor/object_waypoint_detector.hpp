/*
 * object_waypoint_detector.hpp
 *
 *  Created on: Apr 23, 2020
 *      Author: kyberszittya
 */

#ifndef INCLUDE_REI_PERCEPTION_MONITOR_OBJECT_WAYPOINT_DETECTOR_HPP_
#define INCLUDE_REI_PERCEPTION_MONITOR_OBJECT_WAYPOINT_DETECTOR_HPP_

#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>

#include <std_msgs/Int32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point32.h>

#include <visualization_msgs/MarkerArray.h>

#include <eigen_conversions/eigen_msg.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>



#include <hotaru_msgs/RefinedTrajectory.h>
#include <rei_planner_signals/ReplanRequest.h>
#include <rei_monitoring_msgs/DetectedObstacles.h>
#include <rei_common/geometric_utilities.hpp>
// DEPRECATED: our framework should be independent of Autoware
#include <autoware_msgs/DetectedObjectArray.h>



namespace rei
{

constexpr double D_LATERAL_THRESHOLD = 2.75;
constexpr double D_LONGITUDINAL_THRESHOLD = 5.75;

class ObstacleGridMapMonitor
{
private:
	int closest_waypoint;
	// TF related attributes
	const std::string sensor_frame;
	const std::string local_frame;
	const std::string global_frame;
protected:

	geometry_msgs::PoseStamped current_pose;
	ros::NodeHandle nh;
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
	visualization_msgs::MarkerArray obstacles;
	tf2_ros::Buffer tf_buffer;
	std::unique_ptr<tf2_ros::TransformListener> tf_listener;
	geometry_msgs::TransformStamped transform_current_pose;
	geometry_msgs::TransformStamped inv_transform_current_pose;
public:
	ObstacleGridMapMonitor(
			const std::string global_frame,
			const std::string local_frame,
			const std::string sensor_frame,
			ros::NodeHandle& nh): closest_waypoint(-1),
				sensor_frame(sensor_frame),
				global_frame(global_frame),
				local_frame(local_frame), nh(nh){}

	bool init()
	{
		tf_listener = std::unique_ptr<tf2_ros::TransformListener>(
			new tf2_ros::TransformListener(tf_buffer)
		);
		pub_detected_obstacles = nh.advertise<visualization_msgs::MarkerArray>("/viz_filtered_obstacles", 10);
		pub_replan_signal = nh.advertise<rei_planner_signals::ReplanRequest>("/replan_request_sig",10);
		sub_current_pose = nh.subscribe("current_pose", 10, &ObstacleGridMapMonitor::cbCurrentPose, this);
		sub_closest_waypoint = nh.subscribe("/closest_waypoint", 10, &ObstacleGridMapMonitor::cbClosestWaypoint, this);
		sub_base_waypoints = nh.subscribe("input_trajectory", 1, &ObstacleGridMapMonitor::cbBaseWaypoints, this);
		// Sub polygon obstacles
		//pub_detected_obstacles_array = nh.advertise<autoware_msgs::DetectedObjectArray>("/filtered_obstacles_poly", 10);
		pub_local_gridmap = nh.advertise<grid_map_msgs::GridMap>("/local_grid_map", 1);
		pub_detected_obstacles = nh.advertise<rei_monitoring_msgs::DetectedObstacles>("/rei_perception_monitor/detected_obstacles", 1);
		sub_obstacle_poly = nh.subscribe("/detection/lidar_detector/objects", 1, &ObstacleGridMapMonitor::cbPolyObj, this);
		return true;
	}

	static void filterObstacles(const autoware_msgs::DetectedObjectArray::ConstPtr& msg,
			const hotaru_msgs::RefinedTrajectory& msg_base_waypoints,
			const geometry_msgs::TransformStamped transform_current_pose,
			rei_monitoring_msgs::DetectedObstacles& detected_obstacles,
			const int& closest_waypoint);

	void cbPolyObj(const autoware_msgs::DetectedObjectArray::ConstPtr& msg)
	{
		filterObstacles(msg, msg_base_waypoints, transform_current_pose, detected_obstacles, closest_waypoint);
		if (detected_obstacles.obstacles.size()==0)
		{
			request_msg.eval = false;
		}
		else
		{
			// TODO: make a struct
			//request_msg.obstacle_min_lateral_distance = d_lateral;
			//request_msg.obstacle_min_longitudinal_distance = longitudinal_distance;
			request_msg.eval = true;
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
					local_frame, global_frame, ros::Time(0), ros::Duration(1.0));
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
				m.header.frame_id = local_frame;
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

constexpr char DEFAULT_GLOBAL_FRAME[] {"map"};
constexpr char DEFAULT_LOCAL_FRAME[] {"base_link"};
constexpr char DEFAULT_SENSOR_FRAME[] {"velodyne"};

}


#endif /* INCLUDE_REI_PERCEPTION_MONITOR_OBJECT_WAYPOINT_DETECTOR_HPP_ */
