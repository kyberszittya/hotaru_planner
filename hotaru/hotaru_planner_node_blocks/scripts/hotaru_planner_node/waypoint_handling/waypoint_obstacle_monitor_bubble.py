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


class ObstacleDetectorState(object):

    def __init__(self):
        #
        # Inputs
        self.closest_waypoint = 0
        self.global_waypoints = None
        self.obstacles = None
        self.detected_obstacles = []
        self.alive_obstacles = []
        self.current_pose = None
        #
        # Outputs
        self.msg_alive_obstacles = DetectedObstacles()

    def is_initialized(self):
        return self.global_waypoints is not None



class WaypointObstacleMonitor(object):

    def __init__(self, hz, global_frame="map", safety_threshold=1.3):
        self.detector_state = ObstacleDetectorState()
        #
        self.safety_threshold = safety_threshold
        self.hz = hz
        # TF
        self.global_frame = global_frame
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        #
        # Subscriber
        #
        self.sub_reference_waypoint = rospy.Subscriber("base_waypoints", Lane, self.cb_goal_waypoints)
        self.sub_closest_waypoint = rospy.Subscriber("/closest_waypoint", Int32, self.cb_closest_waypoint)
        self.sub_detected_obstacles = rospy.Subscriber("detection/lidar_detector/objects", DetectedObjectArray,
                                                       self.cb_object_detector)
        #
        # Publisher
        #
        self.pub_detected_obstacles = rospy.Publisher("detected_obstacles", DetectedObstacles, queue_size=1)
        self.pub_obstacle_visualization = rospy.Publisher("detected_obstacles/visualization", MarkerArray, queue_size=1)
        # REI
        self.msg_detected_obstacles = DetectedObstacles()
        self.msg_marker_detected_obstacles = MarkerArray()


    def cb_goal_waypoints(self, data):
        self.detector_state.global_waypoints = data

    def linear_distance(self, v0x, v0y, v1x, v1y, px, py):
        nom = np.abs((v1x - v0x) * (v1y - py) - (v0x - px)*(v1y - v0y))
        nz = np.sqrt((v1y - v0y)**2 + (v1x - v0x)**2)
        return nom / nz

    def cb_closest_waypoint(self, data):
        self.detector_state.closest_waypoint = data.data

    def cb_object_detector(self, data):
        if self.detector_state.is_initialized():
            self.sensor_frame = data.header.frame_id
            try:
                # TF
                self.trans_sensor_global = self.tf_buffer.lookup_transform(self.global_frame, self.sensor_frame, rospy.Time())
                detected_obstacles = []
                for o in data.objects:
                    p = PointStamped()
                    p.point.x = o.pose.position.x
                    p.point.y = o.pose.position.y
                    p.point.z = o.pose.position.z
                    p_global = tf2_geometry_msgs.do_transform_point(p, self.trans_sensor_global)
                    mind, mini = 0, np.inf
                    for i in range(len(self.detector_state.global_waypoints.waypoints)):
                        wp_position = self.detector_state.global_waypoints.waypoints[i].pose.pose.position
                        dx = p_global.point.x - wp_position.x
                        dy = p_global.point.y - wp_position.y
                        d_eucl = np.sqrt(dx**2 + dy**2)
                        if (d_eucl < self.safety_threshold):
                            if d_eucl < mind:
                                mind = d_eucl
                    if mind < self.safety_threshold:
                        detected_obstacles.append(o)

                print(len(detected_obstacles), len(data.objects))



            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logerr("TF error")


def main():
    rospy.init_node("waypoint_obstacle_monitor_bubble")
    wp_monitor = WaypointObstacleMonitor(20)
    rospy.spin()

if __name__=="__main__":
    main()