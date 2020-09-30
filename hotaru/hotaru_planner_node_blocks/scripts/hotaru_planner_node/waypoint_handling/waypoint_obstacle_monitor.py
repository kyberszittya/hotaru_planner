import rospy

from rei_monitoring_msgs.msg import DetectedObstacles, Obstacle
from autoware_msgs.msg import Lane, DetectedObjectArray

from std_msgs.msg import Int32

from visualization_msgs.msg import Marker, MarkerArray

import numpy as np

import tf2_ros

from threading import Lock

from geometry_msgs.msg import PointStamped, Point

import tf2_geometry_msgs

class WaypointObstacleMonitor(object):

    def __init__(self, hz, global_frame="map", robot_frame="base_link", min_threshold_radius=1.0, visualize=True):
        """

        :param hz: Detection frequency
        :param global_frame: Global frame which the base_waypoints are stored
        :param min_threshold_radius: Minimal radius in which objects are ignored (presumably inside the vehicle)
        :param visualize: Visualize detection results
        """
        # TF
        self.global_frame = global_frame
        self.robot_frame = robot_frame
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
        # Lock
        self.lock_obstacle = Lock()
        if visualize:
            self.pub_viz_detected_obstacle_marker = rospy.Publisher("/detected_obstacles/visualization", MarkerArray, queue_size=1)
            self.marker_obstacles = MarkerArray()

    def initialize_timers(self):
        self.timer_obstacle_detection = rospy.Timer(rospy.Duration(1.0/self.hz), self.cb_timer_detection)
        if self.visualize:
            self.viz_timer = rospy.Timer(rospy.Duration(1.0/(self.hz/2.0)), self.cb_viz_timer)


    def cb_goal_waypoints(self, data):
        self.global_waypoint = data

    def linear_distance(self, v0x, v0y, v1x, v1y, px, py):
        nom = np.abs((v1x - v0x) * (v1y - py) - (v0x - px)*(v1y - v0y))
        nz = np.sqrt((v1y - v0y)**2 + (v1x - v0x)**2)
        return nom / nz

    def cb_closest_waypoint(self, data):
        self.closest_waypoint = data.data


    def cb_viz_timer(self, event):
        if self.detected_obstacles is not None:
            i = 0

    def cb_object_detector(self, data):

        self.sensor_frame = data.header.frame_id
        self.msg_detected_obstacles.header.frame_id = self.robot_frame
        OBSTACLE_RADIUS = 1.0
        if self.closest_waypoint is not None:
            try:
                self.trans_sensor = self.tf_buffer.lookup_transform(self.robot_frame, self.sensor_frame,  rospy.Time())
                self.trans_global = self.tf_buffer.lookup_transform(self.robot_frame, self.global_frame,  rospy.Time())
                #self.trans_base_global = self.tf_buffer.lookup_transform(self.global_frame, self.robot_frame, rospy.Time())
                self.trans_base_global = self.tf_buffer.lookup_transform(self.global_frame, self.robot_frame, rospy.Time())
                if self.global_waypoint is not None:
                    self.lock_obstacle.acquire()
                    self.detected_obstacles = []
                    for ob in data.objects:

                        o = PointStamped()
                        o.header.frame_id = ob.header.frame_id
                        o.point.x = ob.pose.position.x
                        o.point.y = ob.pose.position.y
                        o.point.z = ob.pose.position.z
                        obstacle_position = tf2_geometry_msgs.do_transform_point(o, self.trans_sensor)
                        p = np.array([obstacle_position.point.x, obstacle_position.point.y])
                        if 0.0 < obstacle_position.point.x < 25.0:
                            for i in range(self.closest_waypoint, min(len(self.global_waypoint.waypoints) - 2, self.closest_waypoint+40)):
                                p0 = PointStamped()
                                p0.header.frame_id = self.global_waypoint.waypoints[i].pose.header.frame_id
                                p0.point.x = self.global_waypoint.waypoints[i].pose.pose.position.x
                                p0.point.y = self.global_waypoint.waypoints[i].pose.pose.position.y
                                p0.point.z = self.global_waypoint.waypoints[i].pose.pose.position.z
                                p1 = PointStamped()
                                p1.header.frame_id = self.global_waypoint.waypoints[i + 1].pose.header.frame_id
                                p1.point.x = self.global_waypoint.waypoints[i + 1].pose.pose.position.x
                                p1.point.y = self.global_waypoint.waypoints[i + 1].pose.pose.position.y
                                p1.point.z = self.global_waypoint.waypoints[i + 1].pose.pose.position.z
                                w0 = tf2_geometry_msgs.do_transform_point(p0, self.trans_global)
                                w1 = tf2_geometry_msgs.do_transform_point(p1, self.trans_global)
                                w = np.array([w0.point.x, w0.point.y])
                                d = self.linear_distance(
                                    w0.point.x, w0.point.y,
                                    w1.point.x, w1.point.y,
                                    obstacle_position.point.x, obstacle_position.point.y)
                                if np.linalg.norm(w - p) > 4.0:
                                    continue
                                if d < OBSTACLE_RADIUS:
                                    print("Detected obstacle: ", d, obstacle_position.point.x)
                                    gl_o = tf2_geometry_msgs.do_transform_point(obstacle_position, self.trans_base_global)
                                    self.detected_obstacles.append(
                                        np.array([obstacle_position.point.x,
                                                  obstacle_position.point.y,
                                                  gl_o.point.x, gl_o.point.y, i]))
                                    break
                    print()
                    self.lock_obstacle.release()
                    # TODO: migrate to thread
                    self.msg_detected_obstacles.obstacles = []
                    for o in self.detected_obstacles:
                        obs = Obstacle()
                        obs.pose.position.x = o[0]
                        obs.pose.position.y = o[1]
                        # Global position
                        obs.global_pose.position.x = o[2]
                        obs.global_pose.position.y = o[3]
                        # Waypoint index
                        obs.closest_waypoint = i # ATTENTION: remove closest waypoint on the other side!
                        obs.radius = 0.7
                        self.msg_detected_obstacles.obstacles.append(obs)
                    self.msg_detected_obstacles.header.stamp = rospy.Time.now()
                    self.pub_detected_obstacles.publish(self.msg_detected_obstacles)
                    # TODO: migrate to visualization thread
                    self.marker_obstacles.markers = []
                    delete_marker = Marker()
                    delete_marker.action = Marker.DELETEALL
                    self.marker_obstacles.markers.append(delete_marker)
                    self.pub_viz_detected_obstacle_marker.publish(self.marker_obstacles)
                    for i, o in enumerate(self.detected_obstacles):
                        m = Marker()
                        p = Point()
                        p.x = o[0]
                        p.y = o[1]
                        m.type = Marker.SPHERE
                        m.scale.x = 1.0
                        m.scale.y = 1.0
                        m.scale.z = 1.0
                        m.color.r = 1.0
                        m.color.g = 0.0
                        m.color.b = 0.0
                        m.color.a = 1.0
                        m.header.frame_id = self.robot_frame
                        m.pose.position = p
                        m.id = i
                        m.ns = "obstacle"
                        self.marker_obstacles.markers.append(m)
                    self.pub_viz_detected_obstacle_marker.publish(self.marker_obstacles)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logerr("TF error")

    def cb_timer_detection(self, event):
        if self.detected_obstacles is not None:
            self.lock_obstacle.acquire()

            self.lock_obstacle.release()




def main():
    rospy.init_node("obstacle_monitor")
    monitor = WaypointObstacleMonitor(10, "map", "base_link", True)
    monitor.initialize_timers()
    rospy.loginfo("All set, publishing obstacles close to the lane")
    rospy.spin()


if __name__=="__main__":
    main()