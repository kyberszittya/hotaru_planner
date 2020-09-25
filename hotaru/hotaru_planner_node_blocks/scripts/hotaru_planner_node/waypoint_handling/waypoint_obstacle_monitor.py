import rospy

from rei_monitoring_msgs.msg import DetectedObstacles, Obstacle
from autoware_msgs.msg import Lane, DetectedObjectArray

from std_msgs.msg import Int32

from visualization_msgs.msg import Marker, MarkerArray

import numpy as np

import tf2_ros

import tf2_geometry_msgs

class WaypointObstacleMonitor(object):

    def __init__(self, hz, global_frame="map", min_threshold_radius=1.0, visualize=True):
        """

        :param hz: Detection frequency
        :param global_frame: Global frame which the base_waypoints are stored
        :param min_threshold_radius: Minimal radius in which objects are ignored (presumably inside the vehicle)
        :param visualize: Visualize detection results
        """
        # TF
        self.global_frame = global_frame
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.trans = None
        # ROS ecosystem
        self.sub_reference_waypoint = rospy.Subscriber("base_waypoints", Lane, self.cb_goal_waypoints)
        self.sub_closest_waypoint = rospy.Subscriber("/closest_waypoint", Int32, self.cb_closest_waypoint)
        self.sub_detected_obstacles = rospy.Subscriber("detection/lidar_detector/objects", DetectedObjectArray, self.cb_object_detector)
        self.pub_detected_obstacles = rospy.Publisher("detected_obstacles", DetectedObstacles, queue_size=1)
        self.hz = hz
        self.visualize = visualize
        self.detected_obstacles = None
        self.global_waypoint = None
        self.sensor_frame = None
        self.closest_waypoint = None
        self.min_object_radius = min_threshold_radius
        # REI
        self.msg_detected_obstacles = DetectedObstacles()

        if visualize:
            self.pub_viz_detected_obstacle_marker = rospy.Publisher("/detected_obstacles/visualization", MarkerArray, queue_size=1)
            self.marker_obstacles = MarkerArray()

    def initialize_timers(self):
        self.timer_obstacle_detection = rospy.Timer(rospy.Duration(1.0/self.hz), self.cb_timer_detection)
        if self.visualize:
            self.viz_timer = rospy.Timer(rospy.Duration(1.0/(self.hz/2.0)), self.cb_viz_timer)


    def cb_goal_waypoints(self, data):
        self.global_waypoint = data

    def linear_distance(self, v0, v1, p):
        nz = np.linalg.norm(v0 - v1)
        nom = (v1[1] - v0[1]) * p[0] - (v1[0] - v0[0]) * p[1] + v1[0] * v0[1] - v1[1] * v0[0]
        return nom / nz

    def cb_closest_waypoint(self, data):
        self.closest_waypoint = data.data


    def cb_viz_timer(self, event):
        if self.detected_obstacles is not None:
            self.marker_obstacles.markers = []
            for o in self.detected_obstacles:
                m = Marker()
                m.pose.position.x = o[0]
                m.pose.position.y = o[1]
                m.type = Marker.SPHERE
                m.scale.x = 0.7
                m.scale.y = 0.7
                m.scale.z = 0.7
                m.color.r = 1.0
                m.color.g = 0.0
                m.color.b = 0.0
                m.color.a = 1.0
                m.header.frame_id = self.sensor_frame
                self.marker_obstacles.markers.append(m)
            self.pub_viz_detected_obstacle_marker.publish(self.marker_obstacles)

    def cb_object_detector(self, data):
        self.detected_obstacles = []
        self.sensor_frame = data.header.frame_id
        self.msg_detected_obstacles.header.frame_id = self.sensor_frame
        if self.closest_waypoint is not None:
            try:
                self.trans = self.tf_buffer.lookup_transform(self.global_frame, self.sensor_frame,  rospy.Time())
                if self.global_waypoint is not None:
                    for o in data.objects:
                        if o.pose.position.x < self.min_object_radius and o.pose.position.y < self.min_object_radius:
                            continue
                        for i in range(self.closest_waypoint + min(len(self.global_waypoint.waypoints) - 1, self.closest_waypoint+40)):
                            w0 = self.global_waypoint.waypoints[i].pose.pose.position
                            v0 = np.array([w0.x, w0.y])
                            w1 = self.global_waypoint.waypoints[i + 1].pose.pose.position
                            v1 = np.array([w1.x, w1.y])
                            glob_o = tf2_geometry_msgs.do_transform_pose(o, self.trans)
                            p = np.array([glob_o.pose.position.x, glob_o.pose.position.y])
                            d = np.abs(self.linear_distance(v0, v1, p))
                            if d < 3.5:
                                self.detected_obstacles.append(np.array([o.pose.position.x, o.pose.position.y]))
                                break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logerr("TF error")


    def cb_timer_detection(self, event):
        if self.detected_obstacles is not None:
            self.msg_detected_obstacles.obstacles = []
            for o in self.detected_obstacles:
                obs = Obstacle()
                obs.pose.position.x = o[0]
                obs.pose.position.y = o[1]
                obs.radius = 0.7

                self.msg_detected_obstacles.obstacles.append(obs)
            self.msg_detected_obstacles.header.stamp = rospy.Time.now()
            self.pub_detected_obstacles.publish(self.msg_detected_obstacles)




def main():
    rospy.init_node("obstacle_monitor")
    monitor = WaypointObstacleMonitor(10, "map", True)
    monitor.initialize_timers()
    rospy.loginfo("All set, publishing obstacles close to the lane")
    rospy.spin()


if __name__=="__main__":
    main()