import rospy

import tf2_ros
import tf2_geometry_msgs

from threading import Lock

from std_msgs.msg import Int32
from autoware_msgs.msg import Lane, Waypoint
from geometry_msgs.msg import Point, PoseStamped, PointStamped

from rei_monitoring_msgs.msg import DetectedObstacles

from hotaru_planner_node.algorithm.dynamic_lane_astar import DynamicLanePolygon, Obstacle, DynamicLaneAstarPlanner
from hotaru_planner_node.algorithm.bezier import RationalBezierCurve
from visualization_msgs.msg import Marker, MarkerArray

from jsk_rviz_plugins.msg import OverlayText

import time

import numpy as np

class RosDynamicLanePlanner(object):

    def __init__(self, hz, lookahead_points=40,
                 global_frame="map",
                 robot_frame="base_link",
                 lane_width=3.0,
                 skip_points=3,
                 lethal_obstacle_val=250.0,
                 non_lethal_val_scale=20.0,
                 sigma_x=3.5,
                 sigma_y=3.0,
                 visualize=False):
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

    def is_initialized(self):
        return self.closest_waypoint is not None

    def cb_current_pose(self, data):
        try:
            self.trans = self.tf_buffer.lookup_transform(self.robot_frame, self.global_frame, rospy.Time())
            self.trans_global = self.tf_buffer.lookup_transform(self.global_frame, self.robot_frame, rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("TF error")
            self.cnt_obstacles = 0
        self.current_pose = data

    def cb_obstacle_detection(self, data):
        if self.is_initialized():
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
                    gl_pos = np.array([o.global_pose.position.x, o.global_pose.position.y])
                    ob = Obstacle(p, o.radius)
                    self.obstacle_deque.append(ob)
                    self.obstacle_list.append([ob, gl_pos, 1])
            # TODO: migrate this part to a time-oriented version
            # if self.cnt_obstacles != len(self.obstacle_list):
        if self.cnt_obstacles != len(self.obstacle_list) and self.current_state == self.states["RELAY"]:
            # TODO: this is temporal
            self.cnt_obstacles = len(self.obstacle_list)
            # self.planning_point = self.closest_waypoint + self.lookahead_points - 14
            print(self.obstacle_list)
            if not self.feasible_plan or self.current_state == self.states["RELAY"]:
                self.event_plan()
            self.current_state = self.states["REPLANNING"]
            if len(self.final_lane.waypoints) > 0:
                self.feasible_plan = True
                self.planning_point = self.search_for_waypoint(self.final_lane.waypoints[-1].pose.pose.position)
            else:
                self.feasible_plan = False

        if self.cnt_obstacles == 0 and self.closest_waypoint > self.planning_point and self.current_state == \
                self.states["REPLANNING"]:
            self.current_state = self.states["RELAY"]
        self.prev_obstacle_event = rospy.Time.now()

    def event_plan(self):
        start_plan = time.clock()
        #
        if len(self.cvs) > 0:
            self.planner_lock.acquire()
            self.current_cvs_2d = np.stack(self.cvs)
            self.planner_lock.release()
            self.lane_polygon.set_reference_trajectory(self.current_cvs_2d)
        else:
            return None
        pose = np.array([0, 0])
        self.planner_lock.acquire()
        self.planner_lock.release()
        _, _, _, self.poly_vertices = self.lane_polygon.calc()
        self.lane_polygon.reset_obstacle_list()
        # Obstacles
        self.lane_polygon.reset_obstacle_list()
        for o in self.obstacle_list:
            self.lane_polygon.add_obstacle(o[0])
        self.planner.set_lane_position(self.planner.find_closest_lane(pose))
        self.coarse_trajectory, weights = self.planner.plan()
        final_cv = self.lane_polygon.get_points(self.coarse_trajectory)
        # Interpolation
        # self.interpolator_final_waypoints.add_control_vertices(final_cv)
        # self.interpolator_final_waypoints.initialize_parameter_values()
        # self.interpolator_final_waypoints.set_weights(weights)
        # self.final_trajectory = self.interpolator_final_waypoints.generate_path(40)
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
        # Orientation calculation
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
            wp.pose = tf2_geometry_msgs.do_transform_pose(pose_local, self.trans_global)
            i_closest_twist = self.closest_ref_waypoint(
                (wp.pose.pose.position.x, wp.pose.pose.position.y),
                current_wps)
            # TODO: this MUST be revised
            if (len(current_wps) == i_closest_twist):
                break
            wp.twist = current_wps[i_closest_twist].twist
            self.final_lane.waypoints.append(wp)
        #
        end_plan = time.clock()
        print("TOTAL PLAN TIME: {0}".format(end_plan - start_plan))



def main():
    rospy.init_node("dynamic_lane_astar")
    dynamic_planner_ros = RosDynamicLanePlanner(10, visualize=True,
                                                skip_points=3,
                                                lethal_obstacle_val=150,
                                                non_lethal_val_scale=100,
                                                sigma_x=7,
                                                sigma_y=0.5)
    dynamic_planner_ros.initialize_timers()
    #while not rospy.is_shutdown():
    #    dynamic_planner_ros.plan()

    rospy.spin()


if __name__=="__main__":
    main()