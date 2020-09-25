from autoware_msgs.msg import Lane, Waypoint

from visualization_msgs.msg import MarkerArray

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

    def __init__(self, hz, visualize=False):
        self.visualize = visualize
        self.pub_waypoint = rospy.Publisher("/base_waypoints", Lane, queue_size=2)
        self.pub_test = rospy.Publisher("/pose", PoseStamped, queue_size=1)
        self.msg_pub_lane = Lane()
        self.msg_pose = PoseStamped()
        self.hz = hz
        if visualize:
            self.pub_visualize = rospy.Publisher("/global_waypoints/visualization", MarkerArray, queue_size=1)
            self.waypoint_marker = MarkerArray()
            self.lane_marker = Marker()
            self.lane_marker.header.frame_id = "map"
            # Color
            self.lane_marker.ns = "waypoint_strip"
            self.lane_marker.color.r = 0.7
            self.lane_marker.color.g = 0.4
            self.lane_marker.color.b = 0.8
            self.lane_marker.color.a = 0.5
            self.lane_marker.scale.x = 1.0
            self.lane_marker.scale.z = 1.0
            self.lane_marker.type = Marker.LINE_STRIP
            #
            self.lane_points = Marker()
            self.lane_points.header.frame_id = "map"
            self.lane_points.ns = "lane_points"
            self.lane_points.type = Marker.SPHERE_LIST
            self.lane_points.color.r = 0.0
            self.lane_points.color.g = 1.0
            self.lane_points.color.b = 1.0
            self.lane_points.color.a = 1.0
            self.lane_points.scale.x = 1.0
            self.lane_points.scale.y = 1.0
            self.lane_points.scale.z = 1.0
            #
            self.lane_points = Marker()
            self.lane_points.header.frame_id = "map"
            self.lane_points.ns = "lane_points"
            self.lane_points.type = Marker.ARROW
            self.lane_points.color.r = 1.0
            self.lane_points.color.g = 0.0
            self.lane_points.color.b = 1.0
            self.lane_points.color.a = 1.0
            self.lane_points.scale.x = 1.0
            self.lane_points.scale.y = 1.0
            self.lane_points.scale.z = 1.0


    def load_csv(self, path):
        waypoint_list = []
        with open(path) as f:
            header = f.readline()
            for line in f:
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
                    self.lane_marker.points.append(p)
                    self.lane_points.points.append(p)
            if self.visualize:
                self.waypoint_marker.markers.append(self.lane_marker)
                self.waypoint_marker.markers.append(self.lane_points)
        rospy.loginfo("Successfully loaded lane description (CSV)")
        self.msg_pub_lane.header.frame_id = "map"
        return waypoint_list

    def initialize_timer(self):
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.hz), self.timer_waypoint, reset=True)
        if self.visualize:
            self.visualization_timer = rospy.Timer(rospy.Duration(1.0/(self.hz/2.0)), self.timer_visualization, reset=True)


    def timer_visualization(self, event):
        self.pub_visualize.publish(self.waypoint_marker)

    def timer_waypoint(self, event):
        self.msg_pub_lane.header.stamp = rospy.Time.now()
        self.pub_waypoint.publish(self.msg_pub_lane)
        self.pub_test.publish(self.msg_pose)



def main():
    rospy.init_node("global_planner_waypoint_publisher")
    global_planner = WaypointLoaderGlobalPlanner(10.0, True)
    global_planner.initialize_timer()
    #waypoint_list = global_planner.load_csv("/home/kyberszittya/zalazone_ws/waypoints/zala_sav_kozep2.csv")
    waypoint_list = global_planner.load_csv("/home/kyberszittya/zalazone_ws/waypoints/smart_city/smart_city_sav_kozep_duro1.csv")
    rospy.loginfo("All set, publishing lane information")
    rospy.spin()



if __name__=="__main__":
    main()
