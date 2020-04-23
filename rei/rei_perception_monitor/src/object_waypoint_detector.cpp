/*
 * object_waypoint_detector.cpp
 *
 *  Created on: Apr 23, 2020
 *      Author: kyberszittya
 */

#include <rei_perception_monitor/object_waypoint_detector.hpp>

namespace rei
{

void ObstacleGridMapMonitor::filterObstacles(const autoware_msgs::DetectedObjectArray::ConstPtr& msg,
			const hotaru_msgs::RefinedTrajectory& msg_base_waypoints,
			const geometry_msgs::TransformStamped transform_current_pose,
			rei_monitoring_msgs::DetectedObstacles& detected_obstacles,
			const int& closest_waypoint)
{
	// Filter obstacles of interest
	double longitudinal_distance = 0.0;
	if (closest_waypoint >= 1)
	{
		detected_obstacles.obstacles.clear();
		detected_obstacles.max_closest_waypoint = 0;

		// Typical application of polytopes!
		// QHULL or CCD should be included or FCL
		// Algorithm is like: check the lateral distance of EVERY object from the global lane
		for (const auto& o: msg->objects)
		{
			if (o.pose.position.x > 1.0)
			{
				for (int i = closest_waypoint; i < msg_base_waypoints.waypoints.size(); i++)
				{
					// Transform waypoints in to the local reference frame
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
					double d_lateral = distanceToLine(_pose_0.pose.position,
							_pose_1.pose.position, o.pose.position);
					if (d_lateral <= D_LATERAL_THRESHOLD)
					{
						///ROS_INFO("Obstacle detected");
						rei_monitoring_msgs::Obstacle o1;
						o1.obstacle_type = rei_monitoring_msgs::Obstacle::STATIC_OBSTACLE;
						o1.radius = 1.2;
						o1.pose = o.pose;
						//tf2::doTransform(o.pose, o1.pose, transform_detector);
						o1.closest_waypoint = i;
						detected_obstacles.max_closest_waypoint = std::max(
								static_cast<int>(detected_obstacles.max_closest_waypoint), i);
						detected_obstacles.obstacles.push_back(std::move(o1));
						break;
					}
					if (rei::isInvalidPoint(_pose_1.pose.position)){ break; }
					longitudinal_distance += planarDistance(
						_pose_0.pose.position,
						_pose_1.pose.position
					);
					if (longitudinal_distance >= 150.0) { break; }
				}
			}
		}

	}

}

}

