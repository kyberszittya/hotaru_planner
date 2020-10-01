
import rospy

from autoware_msgs.msg import Lane, Waypoint

from geometry_msgs.msg import PoseStamped, TwistStamped, Point

from visualization_msgs.msg import MarkerArray, Marker

import rospy

import math


class StructWaypoint(object):

    def __init__(self, x, y, yaw, lin_vel):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.lin_vel = lin_vel


class WaypointLoaderGlobalPlanner(object):

    def __init__(self, hz, visualize=True):
        self.visualize = visualize
        self.pub_waypoint = rospy.Publisher("/base_waypoints", Lane, queue_size=2)
        self.msg_pub_lane = Lane()
        self.msg_pose = PoseStamped()
        self.hz = hz
        if visualize:
            self.pub_visualize_avoiding_trajectory = rospy.Publisher("/global_waypoints/visualization",
                                                                     MarkerArray, queue_size=1, latch=True)
            self.pub_visualize_original_trajectory = rospy.Publisher("/global_waypoints/original/visualization",
                                                                     MarkerArray, queue_size=1, latch=True)
            self.waypoint_marker_avoiding_trajectory = MarkerArray()
            self.waypoint_marker_original_trajectory = MarkerArray()
            # Avoiding trajectory lane strip (visualization)
            self.avoiding_lane_marker = Marker()
            self.avoiding_lane_marker.header.frame_id = "map"
            self.avoiding_lane_marker.ns = "waypoint_strip"
            self.avoiding_lane_marker.color.r = 0.7
            self.avoiding_lane_marker.color.g = 0.4
            self.avoiding_lane_marker.color.b = 0.8
            self.avoiding_lane_marker.color.a = 0.5
            self.avoiding_lane_marker.pose.orientation.w = 1.0
            self.avoiding_lane_marker.scale.x = 0.2
            self.avoiding_lane_marker.scale.z = 0.2
            self.avoiding_lane_marker.type = Marker.LINE_STRIP
            # Original trajectory lane strip (visualization)
            self.original_lane_marker = Marker()
            self.original_lane_marker.header.frame_id = "map"
            self.original_lane_marker.ns = "waypoint_strip"
            self.original_lane_marker.color.r = 0.2
            self.original_lane_marker.color.g = 0.2
            self.original_lane_marker.color.b = 0.2
            self.original_lane_marker.color.a = 0.3
            self.original_lane_marker.pose.orientation.w = 1.0
            self.original_lane_marker.scale.x = 0.1
            self.original_lane_marker.scale.z = 0.2
            self.original_lane_marker.type = Marker.LINE_STRIP

    def load_csv_avoiding(self, path):
        waypoint_list = []
        with open(path) as f:
            header = f.readline()
            for i,line in enumerate(f):
                values = line.strip().split(',')
                x = float(values[0])
                y = float(values[1])
                z = float(values[2])
                yaw = float(values[3])
                lin_vel = float(values[4])/3.6
                wp = StructWaypoint(x, y, yaw, lin_vel)
                waypoint_list.append(wp)
                w0 = Waypoint()
                pose = PoseStamped()
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.position.z = z
                pose.pose.orientation.z = math.sin(yaw/2.0)
                pose.pose.orientation.w = math.cos(yaw/2.0)
                w0.pose = pose
                w0.twist = TwistStamped()
                w0.twist.twist.linear.x = lin_vel
                self.msg_pub_lane.waypoints.append(w0)
                if self.visualize:
                    p = Point()
                    p.x = x
                    p.y = y
                    p.z = z
                    self.avoiding_lane_marker.points.append(p)
                # Orientation
                ori_arrow = Marker()
                ori_arrow.type = Marker.ARROW
                ori_arrow.pose.position.x = x
                ori_arrow.pose.position.y = y
                ori_arrow.pose.position.z = z
                ori_arrow.pose.orientation.z = math.sin(yaw/2.0)
                ori_arrow.pose.orientation.w = math.cos(yaw/2.0)
                ori_arrow.action = Marker.ADD
                ori_arrow.color.r = 1.0
                ori_arrow.color.g = 0.7
                ori_arrow.color.b = 0.6
                ori_arrow.color.a = 1.0
                ori_arrow.scale.x = 1.0
                ori_arrow.scale.y = 0.1
                ori_arrow.id = i
                ori_arrow.ns = "orientation"
                ori_arrow.header.frame_id = "map"
                self.waypoint_marker_avoiding_trajectory.markers.append(ori_arrow)
                # Velocity
                marker_lin_vel = Marker()
                marker_lin_vel.type = Marker.TEXT_VIEW_FACING
                marker_lin_vel.pose.position.x = x
                marker_lin_vel.pose.position.y = y
                marker_lin_vel.pose.position.z = z + 0.7
                marker_lin_vel.ns = "linvel"
                marker_lin_vel.header.frame_id = "map"
                marker_lin_vel.color.r = 1.0
                marker_lin_vel.color.g = 0.7
                marker_lin_vel.color.a = 1.0
                marker_lin_vel.scale.z = 0.5
                marker_lin_vel.id = i
                marker_lin_vel.text = str(round(lin_vel,1))
                self.waypoint_marker_avoiding_trajectory.markers.append(marker_lin_vel)
                # Sphere
                marker_lane_points = Marker()
                marker_lane_points.header.frame_id = "map"
                marker_lane_points.ns = "lane_points"
                marker_lane_points.type = Marker.SPHERE
                marker_lane_points.color.r = 0.0
                marker_lane_points.color.g = 0.7
                marker_lane_points.color.b = 1.0
                marker_lane_points.color.a = 1.0
                marker_lane_points.scale.x = 0.5
                marker_lane_points.scale.y = 0.5
                marker_lane_points.scale.z = 0.5
                marker_lane_points.pose.position = p
                marker_lane_points.pose.orientation.w = 1.0
                marker_lane_points.id = i
                self.waypoint_marker_avoiding_trajectory.markers.append(marker_lane_points)
            if self.visualize:
                self.waypoint_marker_avoiding_trajectory.markers.append(self.avoiding_lane_marker)
        rospy.loginfo("Successfully loaded avoiding lane description (CSV)")
        self.msg_pub_lane.header.frame_id = "map"
        return waypoint_list

    def load_csv_original(self, path):
        waypoint_list = []
        with open(path) as f:
            header = f.readline()
            for i,line in enumerate(f):
                values = line.strip().split(',')
                x = float(values[0])
                y = float(values[1])
                z = float(values[2])
                yaw = float(values[3])
                lin_vel = float(values[4])/3.6
                wp = StructWaypoint(x, y, yaw, lin_vel)
                waypoint_list.append(wp)
                w0 = Waypoint()
                pose = PoseStamped()
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.position.z = z
                pose.pose.orientation.z = math.sin(yaw/2.0)
                pose.pose.orientation.w = math.cos(yaw/2.0)
                w0.pose = pose
                w0.twist = TwistStamped()
                w0.twist.twist.linear.x = lin_vel
                if self.visualize:
                    p = Point()
                    p.x = x
                    p.y = y
                    p.z = z
                    self.original_lane_marker.points.append(p)
                # Orientation
                ori_arrow = Marker()
                ori_arrow.type = Marker.ARROW
                ori_arrow.pose.position.x = x
                ori_arrow.pose.position.y = y
                ori_arrow.pose.position.z = z
                ori_arrow.pose.orientation.z = math.sin(yaw/2.0)
                ori_arrow.pose.orientation.w = math.cos(yaw/2.0)
                ori_arrow.action = Marker.ADD
                ori_arrow.color.r = 0.2
                ori_arrow.color.g = 0.3
                ori_arrow.color.b = 1.0
                ori_arrow.color.a = 0.3
                ori_arrow.scale.x = 1.0
                ori_arrow.scale.y = 0.1
                ori_arrow.id = i
                ori_arrow.ns = "orientation"
                ori_arrow.header.frame_id = "map"
                self.waypoint_marker_original_trajectory.markers.append(ori_arrow)
                # Velocity
                marker_lin_vel = Marker()
                marker_lin_vel.type = Marker.TEXT_VIEW_FACING
                marker_lin_vel.pose.position.x = x
                marker_lin_vel.pose.position.y = y
                marker_lin_vel.pose.position.z = z + 0.7
                marker_lin_vel.ns = "linvel"
                marker_lin_vel.header.frame_id = "map"
                marker_lin_vel.color.r = 1.0
                marker_lin_vel.color.g = 0.7
                marker_lin_vel.color.a = 1.0
                marker_lin_vel.scale.z = 0.5
                marker_lin_vel.id = i
                marker_lin_vel.text = str(round(lin_vel,1))
                self.waypoint_marker_original_trajectory.markers.append(marker_lin_vel)
                # Sphere
                marker_lane_points = Marker()
                marker_lane_points.header.frame_id = "map"
                marker_lane_points.ns = "lane_points"
                marker_lane_points.type = Marker.SPHERE
                marker_lane_points.color.r = 1.0
                marker_lane_points.color.g = 0.7
                marker_lane_points.color.b = 0.0
                marker_lane_points.color.a = 0.5
                marker_lane_points.scale.x = 0.2
                marker_lane_points.scale.y = 0.2
                marker_lane_points.scale.z = 0.2
                marker_lane_points.pose.position = p
                marker_lane_points.pose.orientation.w = 1.0
                marker_lane_points.id = i
                self.waypoint_marker_original_trajectory.markers.append(marker_lane_points)
            if self.visualize:
                self.waypoint_marker_original_trajectory.markers.append(self.original_lane_marker)
        rospy.loginfo("Successfully loaded original lane description (CSV)")
        return waypoint_list

    def initialize_timer(self):
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.hz), self.timer_waypoint, reset=True)
        if self.visualize:
            self.visualization_timer = rospy.Timer(rospy.Duration(1.0/(self.hz/2.0)), self.timer_visualization, reset=True)

    def timer_visualization(self, event):
        self.pub_visualize_avoiding_trajectory.publish(self.waypoint_marker_avoiding_trajectory)
        self.pub_visualize_original_trajectory.publish(self.waypoint_marker_original_trajectory)

    def timer_waypoint(self, event):
        self.msg_pub_lane.header.stamp = rospy.Time.now()
        self.pub_waypoint.publish(self.msg_pub_lane)


def main():
    rospy.init_node("global_planner_avoiding_waypoint_publisher")
    global_planner = WaypointLoaderGlobalPlanner(20, True)
    global_planner.initialize_timer()
    # Avoidance trajectory
    global_planner.load_csv_original("/home/kyberszittya/zalazone_ws/waypoints/demo/avoiding/demo_akadaly_kozep_nova.csv")
    global_planner.load_csv_avoiding("/home/kyberszittya/zalazone_ws/waypoints/demo/avoiding/demo_kozep_sav__kek_vil2_nova.csv")
    # Base (nonavoidance trajectory)

    rospy.loginfo("All set, publishing lane information")
    rospy.spin()


if __name__ == "__main__":
    main()