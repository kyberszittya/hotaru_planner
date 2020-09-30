import rospy

from geometry_msgs.msg import PointStamped, Point

import tf2_geometry_msgs


import numpy as np

import tf2_ros

from rei_monitoring_msgs.msg import DetectedObstacles, Obstacle
from autoware_msgs.msg import Lane, DetectedObjectArray

from visualization_msgs.msg import Marker, MarkerArray


class DummyObstacleMonitor(object):

    def __init__(self, hz, global_frame="map", robot_frame="base_link",
                 min_threshold_radius=1.0, visualize=True):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.sub_detected_obstacles = rospy.Subscriber("detection/lidar_detector/objects", DetectedObjectArray,
                                                       self.cb_object_detector)
        self.pub_detected_obstacles = rospy.Publisher("detected_obstacles", DetectedObstacles, queue_size=1)
        self.hz = hz
        self.visualize = visualize
        self.global_frame = global_frame
        # State
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
        self.trans_global = self.tf_buffer.lookup_transform(
            self.global_frame, data.header.frame_id, rospy.Time())
        self.msg_detected_obstacles.obstacles = []
        for ob in data.objects:
            if ob.pose.position.x < 30.0 and ob.pose.position.y < 2.0 and ob.pose.position.y > -2.0:
                obs = Obstacle()
                # Global position
                p0 = PointStamped()
                p0.point.x = ob.pose.position.x
                p0.point.y = ob.pose.position.y
                p0.point.z = ob.pose.position.z

                gl_o = tf2_geometry_msgs.do_transform_point(ob.pose.position, self.trans_base_global)
                obs.global_pose.position.x = gl_o.point.x
                obs.global_pose.position.y = gl_o.point.y
                obs.radius = 0.7
                self.msg_detected_obstacles.obstacles.append(obs)



def main():
    rospy.init_node("obstacle_monitor")
    monitor = DummyObstacleMonitor(10, "map", "base_link", True)
    rospy.loginfo("All set, publishing obstacles close to the lane")
    rospy.spin()

if __name__=="__main__":
    main()