import rospy

from rei_monitoring_msgs.msg import DetectedObstacles, Obstacle

from autoware_msgs.msg import DetectedObjectArray, Lane

from std_msgs.msg import Int32

from geometry_msgs.msg import PointStamped

import tf2_ros

import tf2_geometry_msgs

import numpy as np

from visualization_msgs.msg import MarkerArray, Marker


class WaypointObstacleMonitor(object):

    def __init__(self):
        # TF
        self.global_frame = "map"
        self.robot_frame = "base_link"
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        # ROS Subscriber
        self.sub_reference_waypoint = rospy.Subscriber("base_waypoints", Lane, self.cb_goal_waypoints)
        self.sub_closest_waypoint = rospy.Subscriber("/closest_waypoint", Int32, self.cb_closest_waypoint)
        self.sub_detected_obstacles = rospy.Subscriber("detection/lidar_detector/objects", DetectedObjectArray,
                                                       self.cb_object_detector)
        self.pub_detected_obstacles = rospy.Publisher("detected_obstacles", DetectedObstacles, queue_size=1)
        self.msg_obstacles = DetectedObstacles()
        #
        self.pub_viz_obstacle_marker = rospy.Publisher("detected_obstacles/visualization", MarkerArray, queue_size=1)
        self.msg_viz_obstacle_markers = MarkerArray()
        #
        self.global_obstacles = []
        self.alive_objects = []

    def cb_closest_waypoint(self, data):
        self.closest_waypoint = data.data

    def cb_goal_waypoints(self, data):
        self.global_waypoint = data

    def cb_object_detector(self, data):
        try:
            self.trans_sensor = self.tf_buffer.lookup_transform(self.robot_frame, data.header.frame_id, rospy.Time())
            self.trans_sensor_global = self.tf_buffer.lookup_transform(self.global_frame, data.header.frame_id, rospy.Time())
            detected_obstacles = []
            for ob in data.objects:
                o = PointStamped()
                o.header.frame_id = ob.header.frame_id
                o.point.x = ob.pose.position.x
                o.point.y = ob.pose.position.y
                o.point.z = ob.pose.position.z
                obstacle_position = tf2_geometry_msgs.do_transform_point(o, self.trans_sensor)
                obstacle_position_global = tf2_geometry_msgs.do_transform_point(o, self.trans_sensor_global)
                p0 = np.array([obstacle_position.point.x, obstacle_position.point.y,
                               obstacle_position_global.point.x, obstacle_position_global.point.y])
                for i,wp in enumerate(self.global_waypoint.waypoints):
                    wp_position = wp.pose.pose.position
                    dx = wp_position.x - p0[2]
                    dy = wp_position.y - p0[3]
                    d = dx**2 + dy**2
                    if d < 0.9:
                        detected_obstacles.append([i,1,p0])
                        break
            # Search for existing obstacles
            for ob in detected_obstacles:
                match = False
                for o in self.global_obstacles:
                    if (o[0] == ob[0] or o[0] + 1 == ob[0] or o[0] - 1 == ob[0]) and \
                            ob[0] > self.closest_waypoint:
                        match=True
                        o[1] += 1
                        if o[1] > 20:
                            o[1] = 20
                if not match:
                    self.global_obstacles.append(ob)
            # Get alive objects
            for o in self.global_obstacles:
                if o[1] > 10 and o[0]:
                    match = False
                    for ob in self.alive_objects:
                        if ob[0]==o[0]:
                            match = True
                            try:
                                if self.closest_waypoint > o[0]:
                                    self.alive_objects.remove(o)
                            except ValueError:
                                self.alive_objects = []
                                self.global_obstacles = []
                    if not match:
                        self.alive_objects.append(o)
            # Check for deacying objects
            for o in self.alive_objects:
                if o[0] < self.closest_waypoint:
                    o[1] -= 1
            # Remove old objects
            for o in self.alive_objects:
                if o[1] <= 0:
                    self.alive_objects.remove(o)
            # Publish alive objects
            print(self.alive_objects)
            self.msg_obstacles.obstacles = []
            # Visualization
            self.msg_viz_obstacle_markers.markers = []
            #
            for i, ob in enumerate(self.alive_objects[0:1]):
                msg_obstacle = Obstacle()
                msg_obstacle.pose.position.x = ob[2][0]
                msg_obstacle.pose.position.y = ob[2][1]
                msg_obstacle.global_pose.position.x = ob[2][2]
                msg_obstacle.global_pose.position.y = ob[2][3]
                msg_obstacle.closest_waypoint = ob[0]
                self.msg_obstacles.obstacles.append(msg_obstacle)
                # Visualization
                # TODO: marker publish
                m = Marker()
                m.header.frame_id = "map"
                m.ns = "obstacle"
                m.id = i
                m.type = Marker.SPHERE
                m.pose.position.x = ob[2][2]
                m.pose.position.y = ob[2][3]
                m.pose.orientation.w = 1.0
                m.scale.x = 1.5
                m.scale.y = 1.5
                m.scale.z = 1.5
                m.color.r = 1.0
                m.color.g = 0.7
                m.color.a = 1.0
                self.msg_viz_obstacle_markers.markers.append(m)
            self.pub_detected_obstacles.publish(self.msg_obstacles)
            self.pub_viz_obstacle_marker.publish(self.msg_viz_obstacle_markers)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("TF error")


def main():
    rospy.init_node("waypoint_obstacle_monitor")
    obstacle_monitor = WaypointObstacleMonitor()
    rospy.spin()


if __name__ == "__main__":
    main()
