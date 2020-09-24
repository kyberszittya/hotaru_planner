import rospy

import tf2_ros
import tf2_geometry_msgs

import threading

from std_msgs.msg import Int64
from autoware_msgs.msg import Lane, Waypoint
from geometry_msgs.msg import Point, PoseStamped

from rei_monitoring_msgs.msg import DetectedObstacles

from hotaru_planner_node.algorithm.dynamic_lane_astar import DynamicLanePolygon, Obstacle, DynamicLaneAstarPlanner
from hotaru_planner_node.algorithm.bezier import RationalBezierCurve
from visualization_msgs.msg import Marker, MarkerArray

import numpy as np

import matplotlib.pyplot as plt

import time

class RosDynamicLanePlanner(object):

    def __init__(self, hz, lookahead_points=40,
                 global_frame="map",
                 robot_frame="base_link",
                 lane_width=3.0,
                 skip_points=3,
                 visualize=False):
        # Locks
        # TF
        self.global_frame = global_frame
        self.robot_frame = robot_frame
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.trans = None
        self.skip_points = skip_points
        #
        self.pub_final_trajectory = rospy.Publisher("/final_waypoints", Lane, queue_size=1)
        # State
        self.poly_vertices = None
        self.current_pose = None
        self.closest_waypoint = None
        self.current_wps = []
        self.current_cvs = []
        self.cvs = []
        self.current_cvs_2d = None
        self.visualize = visualize
        self.obstacle_list = []
        self.hz = hz
        # Store trajectories
        self.coarse_trajectory = None
        self.final_trajectory = None
        self.final_lane = Lane()
        # Subscribers


        self.lane_polygon = DynamicLanePolygon(lane_width, 5, 15, 250)
        self.lookahead_points = lookahead_points
        self.planner = DynamicLaneAstarPlanner(None, self.lane_polygon)
        self.interpolator_final_waypoints = RationalBezierCurve()

        self.sub_current_pose = rospy.Subscriber("/current_pose", PoseStamped, self.cb_current_pose)
        self.sub_current_lane = rospy.Subscriber("/global_waypoints", Lane, self.cb_lane_waypoints)
        self.sub_closest_waypoint = rospy.Subscriber("/closest_waypoint", Int64, self.cb_closest_waypoint)
        #
        self.sub_obstacles = rospy.Subscriber("/detected_obstacles", DetectedObstacles, self.cb_obstacle_detection)
        #
        if visualize:
            # Visualize selected trajectory
            self.pub_viz_selected_trajectory_points = rospy.Publisher("/trajectory_lookahead/visualization", MarkerArray, queue_size=1)
            self.viz_markers = MarkerArray()
            self.viz_marker = Marker()
            self.viz_marker.header.frame_id = self.robot_frame
            self.viz_marker.type = Marker.LINE_STRIP
            self.viz_marker.color.r = 1.0
            self.viz_marker.color.g = 1.0
            self.viz_marker.color.a = 0.5
            self.viz_marker.scale.x = 2.0
            # Visualize tesselation grid
            self.pub_viz_tesselation_points = rospy.Publisher("/tesselation_grid/visualization",
                                                                      Marker, queue_size=1)
            self.viz_marker_tesselation = Marker()
            self.viz_marker_tesselation.type = Marker.SPHERE_LIST
            self.viz_marker_tesselation.color.r = 1.0
            self.viz_marker_tesselation.color.b = 1.0
            self.viz_marker_tesselation.color.a = 0.4
            self.viz_marker_tesselation.scale.x = 0.2
            self.viz_marker_tesselation.scale.y = 0.2
            self.viz_marker_tesselation.scale.z = 0.2
            self.viz_marker_tesselation.header.frame_id = self.robot_frame
            # Visualize current lane
            self.pub_viz_current_lane_point = rospy.Publisher("/current_lane/visualization", Marker, queue_size=1)
            self.viz_current_lane_marker = Marker()
            self.viz_current_lane_marker.type = Marker.SPHERE
            self.viz_current_lane_marker.color.r = 1.0
            self.viz_current_lane_marker.color.b = 1.0
            self.viz_current_lane_marker.color.a = 0.4
            self.viz_current_lane_marker.scale.x = 1.5
            self.viz_current_lane_marker.scale.y = 1.5
            self.viz_current_lane_marker.scale.z = 1.5
            self.viz_current_lane_marker.header.frame_id = self.robot_frame
            # Final trajectory
            self.pub_viz_final_trajectory = rospy.Publisher("/final_trajectory/visualization", MarkerArray, queue_size=1)
            self.viz_marker_array_final_trajectory = MarkerArray()
            self.viz_marker_final_trajectory = Marker()
            self.viz_marker_final_trajectory.type = Marker.LINE_STRIP
            self.viz_marker_final_trajectory.color.r = 0.4
            self.viz_marker_final_trajectory.color.g = 0.8
            self.viz_marker_final_trajectory.color.b = 1.0
            self.viz_marker_final_trajectory.color.a = 1.0
            self.viz_marker_final_trajectory.scale.x = 1.7
            self.viz_marker_final_trajectory.header.frame_id = self.robot_frame
            self.viz_marker_array_final_trajectory.markers.append(self.viz_marker_final_trajectory)

    def initialize_timers(self):
        #self.timer_planner = rospy.Timer(rospy.Duration(1.0/self.hz), self.cb_plan_timer)
        if self.visualize:
            self.timer_visualization = rospy.Timer(rospy.Duration(1.0/(self.hz/2.0)), self.cb_visualize)

    def cb_current_pose(self, data):
        try:
            self.trans = self.tf_buffer.lookup_transform(self.robot_frame, self.global_frame, rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("TF error")
        self.current_pose = data

    def cb_obstacle_detection(self, data):
        self.obstacle_list = []
        for i, o in enumerate(data.obstacles):
            p = np.array([o.pose.position.x, o.pose.position.y])
            self.obstacle_list.append(Obstacle(p, o.radius))



    def cb_lane_waypoints(self, data):
        if self.closest_waypoint is not None:
            self.lanes = data
            # TODO: set trajectory
            self.current_wps = []
            self.current_cvs = []
            self.cvs = []
            point_indices = []
            for i in range(self.closest_waypoint,
                           (self.closest_waypoint + self.lookahead_points) % len(data.waypoints),
                           self.skip_points):
                point_indices.append(i)
            if self.trans is not None:
                for i in point_indices:
                    self.current_wps.append(data.waypoints[i])
                    tr_pose = tf2_geometry_msgs.do_transform_pose(data.waypoints[i].pose, self.trans)
                    self.current_cvs.append(tr_pose)
                    self.cvs.append(np.array([tr_pose.pose.position.x, tr_pose.pose.position.y]))

    def cb_closest_waypoint(self, data):
        self.closest_waypoint = data.data

    def cb_visualize(self, event):
        self.viz_markers.markers = []
        self.viz_marker.points = []
        for cv in self.current_cvs:
            p = Point()
            p.x = cv.pose.position.x
            p.y = cv.pose.position.y
            p.z = cv.pose.position.z
            self.viz_marker.points.append(p)
        self.viz_markers.markers.append(self.viz_marker)
        self.pub_viz_selected_trajectory_points.publish(self.viz_markers)
        self.viz_marker_tesselation.points = []
        if self.poly_vertices is not None:
            for t in self.poly_vertices:
                for s in t:
                    p = Point()
                    p.x, p.y = s[0], s[1]
                    self.viz_marker_tesselation.points.append(p)
            self.viz_current_lane_marker.pose.position.x = self.poly_vertices[0, self.planner.current_lane, 0]
            self.viz_current_lane_marker.pose.position.y = self.poly_vertices[0, self.planner.current_lane, 1]
        self.pub_viz_tesselation_points.publish(self.viz_marker_tesselation)
        self.pub_viz_current_lane_point.publish(self.viz_current_lane_marker)
        if self.final_trajectory is not None:
            self.viz_marker_final_trajectory.points = []
            for p in self.final_trajectory:
                p0 = Point()
                p0.x = p[0]
                p0.y = p[1]
                self.viz_marker_final_trajectory.points.append(p0)
            self.pub_viz_final_trajectory.publish(self.viz_marker_array_final_trajectory)

    def cb_plan_timer(self, event):
        self.plan()

    def closest_ref_waypoint(self, p):
        mini, mindist = 0, 100000
        for i,wp in enumerate(self.current_wps):
            d = (p[0]-wp.pose.pose.position.x)**2 + (p[1]-wp.pose.pose.position.x)**2 < mindist
            if d < mindist:
                mini, mindi = i, np.sqrt(d)
        return mini

    def plan(self):
        start_plan = time.clock()
        if len(self.cvs) > 0:
            self.current_cvs_2d = np.stack(self.cvs)
            self.lane_polygon.set_reference_trajectory(self.current_cvs_2d)
        else:
            return None
        pose = np.array([0, 0])
        obs_list = [x for x in self.obstacle_list]
        _, _, _, self.poly_vertices = self.lane_polygon.calc()
        self.lane_polygon.reset_obstacle_list()
        for o in obs_list:
            self.lane_polygon.add_obstacle(o)
        self.planner.set_lane_position(self.planner.find_closest_lane(pose))
        self.coarse_trajectory, weights = self.planner.plan()
        final_cv = self.lane_polygon.get_points(self.coarse_trajectory)
        self.interpolator_final_waypoints.add_control_vertices(final_cv)
        self.interpolator_final_waypoints.initialize_parameter_values()
        self.interpolator_final_waypoints.set_weights(weights)
        self.final_trajectory = self.interpolator_final_waypoints.generate_path(20)
        self.final_lane.waypoints = []
        for i in range(len(self.final_trajectory) - 1):
            d = self.final_trajectory[i + 1] - self.final_trajectory[i]
            d /= np.linalg.norm(d)
            # Search for the closest reference waypoint for velocity
            wp = Waypoint()
            yaw = np.arctan2(d[1], d[0])
            wp.pose.pose.orientation.x = 0.0
            wp.pose.pose.orientation.y = 0.0
            wp.pose.pose.orientation.z = np.sin(yaw / 2.0)
            wp.pose.pose.orientation.w = np.cos(yaw / 2.0)
            wp.pose.pose.position.x = self.final_trajectory[i, 0]
            wp.pose.pose.position.y = self.final_trajectory[i, 1]
            i_closest_twist = self.closest_ref_waypoint(self.final_trajectory[i])
            print(i_closest_twist, len(self.current_wps))
            #wp.twist.twist = self.current_wps[i_closest_twist].twist
            self.final_lane.waypoints.append(wp)
        end_plan = time.clock()
        #print("TOTAL PLAN TIME: {0}".format(end_plan - start_plan))




def main():
    rospy.init_node("dynamic_lane_astar")
    dynamic_planner_ros = RosDynamicLanePlanner(2, visualize=True)
    dynamic_planner_ros.initialize_timers()
    while not rospy.is_shutdown():
        dynamic_planner_ros.plan()

    #rospy.spin()


if __name__=="__main__":
    main()