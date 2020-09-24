import rospy

from rei_monitoring_msgs.msg import DetectedObstacles, Obstacle
from autoware_msgs.msg import Lane, DetectedObjectArray

from visualization_msgs.msg import Marker, MarkerArray

import numpy as np

class WaypointObstacleMonitor(object):

    def __init__(self, hz, visualize =True):
        self.sub_reference_waypoint = rospy.Subscriber("global_waypoints", Lane, self.cb_goal_waypoints)
        self.sub_detected_obstacles = rospy.Subscriber("detection/lidar_detector/objects", DetectedObjectArray, self.cb_object_detector)
        self.pub_detected_obstacles = rospy.Publisher("detected_obstacles", DetectedObstacles, queue_size=1)
        self.hz = hz
        self.visualize = visualize
        self.detected_obstacles = None
        self.global_waypoint = None
        self.sensor_frame = None
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


    def cb_viz_timer(self, event):
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
        if self.global_waypoint is not None:
            for o in data.objects:
                for i in range(len(self.global_waypoint.waypoints) - 1):
                    w0 = self.global_waypoint.waypoints[i].pose.pose.position
                    v0 = np.array([w0.x, w0.y])
                    w1 = self.global_waypoint.waypoints[i + 1].pose.pose.position
                    v1 = np.array([w1.x, w1.y])
                    p = np.array([o.pose.position.x, o.pose.position.y])
                    d = self.linear_distance(v0, v1, p)
                    if d < 3.0:
                        self.detected_obstacles.append(np.array([o.pose.position.x, o.pose.position.y]))
                        break

    def cb_timer_detection(self, event):
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
    monitor = WaypointObstacleMonitor(10, True)
    monitor.initialize_timers()
    rospy.loginfo("All set, publishing obstacles close to the lane")
    rospy.spin()


if __name__=="__main__":
    main()