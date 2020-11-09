import rospy

import tf2_ros

from threading import Lock

from std_msgs.msg import Int32
from autoware_msgs.msg import Lane, Waypoint
from geometry_msgs.msg import Point, PoseStamped, PointStamped

from rei_monitoring_msgs.msg import DetectedObstacles

from hotaru_planner_node.algorithm.dynamic_lane_astar import DynamicLanePolygon, Obstacle, DynamicLaneAstarPlanner
from hotaru_planner_node.algorithm.bezier import RationalBezierCurve
from visualization_msgs.msg import Marker, MarkerArray

from jsk_rviz_plugins.msg import OverlayText

import numpy as np


import time

from collections import deque


class RosDynamicLanePlanner(object):

    def __init__(self, hz, lookahead_points=40,
                 global_frame="map",
                 robot_frame="base_link",
                 lane_width=3.0,
                 skip_points=3,
                 lethal_obstacle_val=250.0,
                 non_lethal_val_scale=20.0,
                 sigma_x = 3.5,
                 sigma_y = 3.0,
                 visualize=False):
        # Locks
        self.planner_lock = Lock()
        self.obstacle_lock = Lock()
        # TF
        self.global_frame = global_frame
        self.robot_frame = robot_frame
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.trans = None
        self.trans_global = None
        self.skip_points = skip_points
        #
        self.pub_final_trajectory = rospy.Publisher("/final_waypoints", Lane, queue_size=1)
        # State
        self.cnt_obstacles = 0
        self.lanes = None
        self.poly_vertices = None
        self.current_pose = None
        self.closest_waypoint = None
        self.current_wps = []
        self.current_cvs = []
        self.cvs = []
        self.current_cvs_2d = None
        self.visualize = visualize
        self.hz = hz
        self.planning_point = 0
        self.feasible_plan = False
        # Obstacle
        self.prev_obstacle_event = rospy.Time.now()

        self.states = {"RELAY": 0, "REPLANNING": 1, "AVOID": 2}
        self.ref_lane_length = 0
        self.current_state = self.states["RELAY"]
        # Store trajectories
        self.coarse_trajectory = None
        self.final_trajectory = None
        self.final_lane = Lane()
        self.obstacle_list = []
        # Subscribers

        # TODO: Hybrid state machine
        self.lane_polygon = DynamicLanePolygon(lane_width, 7, 15,
                                               lethal_val_scale=lethal_obstacle_val,
                                               non_lethal_val_scale=non_lethal_val_scale,
                                               sigma_x=sigma_x, sigma_y=sigma_y)
        self.lookahead_points = lookahead_points
        self.planner = DynamicLaneAstarPlanner(None, self.lane_polygon)
        self.interpolator_final_waypoints = RationalBezierCurve()

        self.sub_current_pose = rospy.Subscriber("/current_pose", PoseStamped, self.cb_current_pose)
        self.sub_current_lane = rospy.Subscriber("/base_waypoints", Lane, self.cb_lane_waypoints)
        self.sub_closest_waypoint = rospy.Subscriber("/closest_waypoint", Int32, self.cb_closest_waypoint)
        #
        self.sub_obstacles = rospy.Subscriber("/detected_obstacles", DetectedObstacles, self.cb_obstacle_detection)
        # Obstacle deque
        self.obstacle_deque = deque()
        # State report
        self.msg_state_text = OverlayText()
        self.msg_state_text.text_size = 14
        self.msg_state_text.width = 800
        self.msg_state_text.height = 100
        self.msg_state_text.bg_color.a = 0.5
        self.msg_state_text.font = "Noto Mono"
        self.pub_state_text = rospy.Publisher("hotaru/state_text", OverlayText, queue_size=1)
        # Handle visualization
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
            self.viz_marker.scale.x = 0.6
            # Visualize current lane
            self.pub_viz_current_lane_point = rospy.Publisher("/current_lane/visualization", Marker, queue_size=1)
            self.viz_current_lane_marker = Marker()
            self.viz_current_lane_marker.type = Marker.SPHERE
            self.viz_current_lane_marker.color.r = 1.0
            self.viz_current_lane_marker.color.b = 1.0
            self.viz_current_lane_marker.color.a = 0.4
            self.viz_current_lane_marker.scale.x = 0.5
            self.viz_current_lane_marker.scale.y = 0.5
            self.viz_current_lane_marker.scale.z = 0.5
            self.viz_current_lane_marker.header.frame_id = self.robot_frame
            # Final trajectory
            self.pub_viz_final_trajectory = rospy.Publisher("/final_trajectory/visualization", MarkerArray, queue_size=1)
            self.viz_marker_array_final_trajectory = MarkerArray()
            # Visualize orientation
            self.viz_marker_orientation_field = Marker()

            self.viz_marker_orientation_field.type = Marker.LINE_LIST
            self.viz_marker_orientation_field.ns = "orientation_field"
            self.viz_marker_orientation_field.color.r = 0.2
            self.viz_marker_orientation_field.color.g = 0.9
            self.viz_marker_orientation_field.color.b = 0.4
            self.viz_marker_orientation_field.color.a = 0.95
            self.viz_marker_orientation_field.scale.x = 0.4
            self.viz_marker_orientation_field.header.frame_id = self.global_frame
            self.viz_marker_array_final_trajectory.markers.append(self.viz_marker_orientation_field)
            # Tesselation visualization
            self.viz_marker_tesselation = MarkerArray()
            # Visualize tesselation grid
            self.pub_viz_tesselation_points = rospy.Publisher("/tesselation_grid/visualization",
                                                              MarkerArray, queue_size=1)

    def initialize_timers(self):
        self.timer_planner = rospy.Timer(rospy.Duration(1.0/self.hz), self.cb_plan_timer, reset=True)
        if self.visualize:
            self.timer_visualization = rospy.Timer(rospy.Duration(1.0/(self.hz/2.0)), self.cb_visualize, reset=True)

    def cb_current_pose(self, data):
        try:
            self.trans = self.tf_buffer.lookup_transform(self.robot_frame, self.global_frame, rospy.Time())
            self.trans_global = self.tf_buffer.lookup_transform(self.global_frame, self.robot_frame, rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("TF error")
            self.cnt_obstacles = 0
        self.current_pose = data

    def search_for_waypoint(self, position):
        mini, mind = 0, np.inf
        for i, wp in enumerate(self.lanes.waypoints):
            dx = position.x - wp.pose.pose.position.x
            dy = position.y - wp.pose.pose.position.y
            d = dx ** 2 + dy ** 2
            if d < mind:
                mini, mind = i, d
        return mini

    def cb_obstacle_detection(self, data):
        if self.is_initialized():
            self.obstacle_list = []
            for i, o in enumerate(data.obstacles):
                gl_pos = np.array([o.global_pose.position.x, o.global_pose.position.y])
                ob = Obstacle(gl_pos, o.radius)
                self.obstacle_deque.append(ob)
                self.obstacle_list.append([ob, gl_pos, 1])
                if not self.feasible_plan or self.current_state == self.states["RELAY"]:
                    self.event_plan()
                    self.current_state = self.states["REPLANNING"]
                if len(self.final_lane.waypoints) > 0:
                    self.feasible_plan = True
                    self.planning_point = self.search_for_waypoint(self.final_lane.waypoints[-1].pose.pose.position)
                else:
                    self.feasible_plan = False
            if self.closest_waypoint > self.planning_point \
                    and self.current_state == self.states["REPLANNING"]:
                self.current_state = self.states["RELAY"]

            """
            if len(data.obstacles) == 0:
                for o in self.obstacle_list:
                    o[2] -= 1
                sum_decay = 0
                for o in self.obstacle_list:
                    sum_decay += o[2]
                if (sum_decay <= 0):
                    self.lane_polygon.reset_obstacle_list()
                    self.cnt_obstacles = 0
                    self.obstacle_list = []
            else:
                for i, o in enumerate(data.obstacles):
                    match = False
                    for gl_o in self.obstacle_list:
                        if np.abs(gl_o[1][0] - o.global_pose.position.x) < 0.5 and \
                                np.abs(gl_o[1][1] - o.global_pose.position.y) < 0.5:
                            match = True
                            gl_o[2] += 1
                            if gl_o[2] > 40:
                                gl_o[2] = 40
                    if not match:
                        p = np.array([o.pose.position.x, o.pose.position.y])
                        gl_pos = np.array([o.global_pose.position.x,  o.global_pose.position.y])
                        ob = Obstacle(p, o.radius)
                        self.obstacle_deque.append(ob)
                        self.obstacle_list.append([ob, gl_pos, 1])
            # TODO: migrate this part to a time-oriented version
            #if self.cnt_obstacles != len(self.obstacle_list):
            if self.cnt_obstacles != len(self.obstacle_list) and self.current_state==self.states["RELAY"]:
                # TODO: this is temporal
                self.cnt_obstacles = len(self.obstacle_list)
                #self.planning_point = self.closest_waypoint + self.lookahead_points - 14
                print(self.obstacle_list)
                if not self.feasible_plan or self.current_state==self.states["RELAY"]:
                    self.event_plan()
                self.current_state = self.states["REPLANNING"]
                if len(self.final_lane.waypoints) > 0:
                    self.feasible_plan = True
                    self.planning_point = self.search_for_waypoint(self.final_lane.waypoints[-1].pose.pose.position)
                else:
                    self.feasible_plan = False

            if self.cnt_obstacles == 0 and self.closest_waypoint > self.planning_point and self.current_state==self.states["REPLANNING"]:
                self.current_state = self.states["RELAY"]
            self.prev_obstacle_event = rospy.Time.now()
            """

    def cb_lane_waypoints(self, data):
        if self.is_initialized():
            self.lanes = data
            self.ref_lane_length = len(self.lanes.waypoints)
            # TODO: set trajectory
            self.current_wps = []
            self.current_cvs = []
            point_indices = []
            for i in range(self.closest_waypoint,
                           (self.closest_waypoint + self.lookahead_points) % len(data.waypoints),
                           self.skip_points):
                point_indices.append(i)
            if self.trans is not None:
                self.planner_lock.acquire()
                self.cvs = []
                for i in point_indices:
                    self.current_wps.append(data.waypoints[i])
                    #tr_pose = tf2_geometry_msgs.do_transform_pose(data.waypoints[i].pose, self.trans)
                    tr_pose = data.waypoints[i].pose
                    self.current_cvs.append(tr_pose)
                    self.cvs.append(np.array([tr_pose.pose.position.x, tr_pose.pose.position.y]))
                self.planner_lock.release()

    def cb_closest_waypoint(self, data):
        self.closest_waypoint = data.data

    def is_initialized(self):
        return self.closest_waypoint is not None

    def cb_visualize(self, event):
        self.viz_marker_array_final_trajectory.markers = []
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
        self.viz_marker_tesselation.markers = []
        viz_tesselation = Marker()
        viz_tesselation.action = Marker.DELETEALL
        self.viz_marker_tesselation.markers.append(viz_tesselation)
        self.pub_viz_tesselation_points.publish(self.viz_marker_tesselation)

        #self.trans_base_global = self.tf_buffer.lookup_transform(self.global_frame, self.robot_frame, rospy.Time())
        if self.poly_vertices is not None:
            for i,t in enumerate(self.poly_vertices):
                for j,s in enumerate(t):
                    p = PointStamped()
                    p.point.x, p.point.y = s[0], s[1]
                    p.header.frame_id = "base_link"
                    id = i * self.lane_polygon.steps_width + j
                    viz_tesselation = Marker()
                    viz_tesselation.type = Marker.SPHERE
                    viz_tesselation.ns = "tesselation"
                    viz_tesselation.id = id
                    #gl_o = tf2_geometry_msgs.do_transform_point(p, self.trans_base_global)
                    if self.lane_polygon.obstacle_grid is None:
                        viz_tesselation.color.r = 1.0
                        viz_tesselation.color.b = 1.0
                        viz_tesselation.color.a = 0.4
                    else:
                        obstacle_val = self.lane_polygon.obstacle_grid[i, j]
                        viz_tesselation.color.r = min(obstacle_val, 1.0)
                        viz_tesselation.color.g = 1.0 / (obstacle_val+1)
                        viz_tesselation.color.a = 1.0
                    #viz_tesselation.pose.position = gl_o.point
                    viz_tesselation.pose.position = p.point
                    viz_tesselation.scale.x = 0.5
                    viz_tesselation.scale.y = 0.5
                    viz_tesselation.scale.z = 0.5
                    viz_tesselation.header.frame_id = self.global_frame
                    self.viz_marker_tesselation.markers.append(viz_tesselation)
            self.viz_current_lane_marker.pose.position.x = self.poly_vertices[0, self.planner.current_lane, 0]
            self.viz_current_lane_marker.pose.position.y = self.poly_vertices[0, self.planner.current_lane, 1]
        self.pub_viz_tesselation_points.publish(self.viz_marker_tesselation)
        self.pub_viz_current_lane_point.publish(self.viz_current_lane_marker)
        if self.final_lane is not None:
            self.viz_marker_orientation_field.points = []
            # Points
            if self.current_state==self.states["REPLANNING"]:
                viz_marker_final_trajectory = Marker()
                viz_marker_final_trajectory.action = Marker.DELETEALL
                self.viz_marker_array_final_trajectory.markers.append(viz_marker_final_trajectory)
                self.pub_viz_final_trajectory.publish(self.viz_marker_array_final_trajectory)
                for i, w in enumerate(self.final_lane.waypoints):
                    viz_marker_final_trajectory = Marker()
                    viz_marker_final_trajectory.ns = "final_trajectory_points"
                    viz_marker_final_trajectory.type = Marker.SPHERE
                    viz_marker_final_trajectory.pose.position = w.pose.pose.position
                    viz_marker_final_trajectory.color.r = 0.4
                    viz_marker_final_trajectory.color.g = 0.8
                    viz_marker_final_trajectory.color.b = 1.0
                    viz_marker_final_trajectory.color.a = 0.8
                    viz_marker_final_trajectory.scale.x = 0.8
                    viz_marker_final_trajectory.scale.y = 0.8
                    viz_marker_final_trajectory.scale.z = 0.8
                    viz_marker_final_trajectory.header.frame_id = self.global_frame
                    viz_marker_final_trajectory.id = i
                    self.viz_marker_array_final_trajectory.markers.append(viz_marker_final_trajectory)
                    #
                    viz_marker_arrow = Marker()
                    viz_marker_arrow.ns = "orientation_arrow"
                    viz_marker_arrow.type = Marker.ARROW
                    viz_marker_arrow.pose.position = w.pose.pose.position
                    viz_marker_arrow.pose.orientation = w.pose.pose.orientation
                    viz_marker_arrow.color.r = 0.8
                    viz_marker_arrow.color.g = 0.4
                    viz_marker_arrow.color.b = 1.0
                    viz_marker_arrow.color.a = 1.0
                    viz_marker_arrow.scale.x = 0.9
                    viz_marker_arrow.scale.y = 0.1
                    viz_marker_arrow.scale.z = 0.1
                    viz_marker_arrow.header.frame_id = self.global_frame
                    viz_marker_arrow.id = i
                    self.viz_marker_array_final_trajectory.markers.append(viz_marker_arrow)
                self.pub_viz_final_trajectory.publish(self.viz_marker_array_final_trajectory)

    def cb_plan_timer(self, event):
        #self.plan()
        if self.lanes is not None:
            # TODO: well, this should be revised
            #if self.closest_waypoint < len(self.lanes.waypoints) - self.lookahead_points and \
            #        not self.isRelay():
            if self.current_state == self.states["REPLANNING"]:
                #self.current_state = self.states["REPLANNING"]
                self.msg_state_text.text = "HOTARU_LOCAL_PLANNER: REPLANNING"
                self.msg_state_text.fg_color.r = 0.0
                self.msg_state_text.fg_color.g = 1.0
                self.msg_state_text.fg_color.b = 1.0
                self.msg_state_text.fg_color.a = 1.0
                #self.plan()
            elif self.current_state == self.states["RELAY"]:
                #self.current_state = self.states["RELAY"]
                self.msg_state_text.text = "HOTARU_LOCAL_PLANNER: RELAY"
                self.msg_state_text.fg_color.r = 0.0
                self.msg_state_text.fg_color.g = 1.0
                self.msg_state_text.fg_color.b = 0.0
                self.msg_state_text.fg_color.a = 1.0
                self.relay()
            self.pub_final_trajectory.publish(self.final_lane)
            # State report
            self.pub_state_text.publish(self.msg_state_text)

    def closest_ref_waypoint(self, p, current_wps):
        mini, mindist = 0, np.inf
        for i,wp in enumerate(current_wps):
            d = (p[0]-wp.pose.pose.position.x)**2 + (p[1]-wp.pose.pose.position.y)**2
            if d < mindist:
                mini, mindi = i, np.sqrt(d)
        return mini

    def relay(self):
        self.final_lane.waypoints = []
        for i in range(self.closest_waypoint, len(self.lanes.waypoints)):
            self.final_lane.waypoints.append(self.lanes.waypoints[i])

    def event_plan(self):
        start_plan = time.clock()
        if len(self.cvs) > 0:
            self.planner_lock.acquire()
            self.current_cvs_2d = np.stack(self.cvs)
            self.planner_lock.release()
            self.lane_polygon.set_reference_trajectory(self.current_cvs_2d)
        else:
            return None
        pose = np.array([self.current_pose.pose.position.x, self.current_pose.pose.position.y])
        self.planner_lock.acquire()
        self.planner_lock.release()
        _, _, _, self.poly_vertices = self.lane_polygon.calc()
        # Obstacles
        self.lane_polygon.reset_obstacle_list()
        for o in self.obstacle_list:
            self.lane_polygon.add_obstacle(o[0])
        self.planner.set_lane_position(self.planner.find_closest_lane(pose))
        self.coarse_trajectory, weights = self.planner.plan()
        final_cv = self.lane_polygon.get_points(self.coarse_trajectory)
        # Interpolation
        #self.interpolator_final_waypoints.add_control_vertices(final_cv)
        #self.interpolator_final_waypoints.initialize_parameter_values()
        #self.interpolator_final_waypoints.set_weights(weights)
        #self.final_trajectory = self.interpolator_final_waypoints.generate_path(40)
        self.final_trajectory = final_cv
        self.final_lane.waypoints = []
        self.planner_lock.acquire()
        current_wps = [wp for wp in self.current_wps]
        self.planner_lock.release()
        # Set current pose as first trajectory
        wp = Waypoint()
        wp.pose = self.current_pose
        wp.pose.header.frame_id = "map"
        self.viz_marker_array_final_trajectory.markers = []
        for i in range(len(self.final_trajectory) - 1):
            d = self.final_trajectory[i + 1] - self.final_trajectory[i]
            d /= np.linalg.norm(d)
            # Search for the closest reference waypoint for velocity
            wp = Waypoint()
            yaw = np.arctan2(d[1], d[0])
            pose_local = PoseStamped()
            pose_local.header.frame_id = self.robot_frame
            pose_local.pose.position.x, pose_local.pose.position.y = self.final_trajectory[i, 0], self.final_trajectory[
                i, 1]
            pose_local.pose.orientation.x = 0.0
            pose_local.pose.orientation.y = 0.0
            pose_local.pose.orientation.z = np.sin(yaw / 2.0)
            pose_local.pose.orientation.w = np.cos(yaw / 2.0)
            #wp.pose = tf2_geometry_msgs.do_transform_pose(pose_local, self.trans_global)
            wp.pose = pose_local
            i_closest_twist = self.closest_ref_waypoint(
                (wp.pose.pose.position.x, wp.pose.pose.position.y),
                current_wps)
            # TODO: this MUST be revised
            if (len(current_wps) == i_closest_twist):
                break
            wp.twist = current_wps[i_closest_twist].twist
            self.final_lane.waypoints.append(wp)
        end_plan = time.clock()
        print("TOTAL PLAN TIME: {0}".format(end_plan - start_plan))




def main():
    rospy.init_node("dynamic_lane_astar")
    dynamic_planner_ros = RosDynamicLanePlanner(10, visualize=True,
                                                skip_points=3,
                                                lethal_obstacle_val=50,
                                                non_lethal_val_scale=35,
                                                sigma_x=12,
                                                sigma_y=4)
    dynamic_planner_ros.initialize_timers()
    #while not rospy.is_shutdown():
    #    dynamic_planner_ros.plan()

    rospy.spin()


if __name__=="__main__":
    main()