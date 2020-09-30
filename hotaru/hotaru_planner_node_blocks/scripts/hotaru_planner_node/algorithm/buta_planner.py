import rospy

from autoware_msgs.msg import Lane, Waypoint

from std_msgs.msg import Int32

from rei_monitoring_msgs.msg import DetectedObstacles

from geometry_msgs.msg import PointStamped, Point, PoseStamped

from visualization_msgs.msg import MarkerArray, Marker

import tf2_ros

import numpy as np

def quaternion_to_yaw(qw, qx, qy, qz):
    x = 2*(qw*qz + qx*qy)
    y = 1 - 2*(qy**2+qz**2)
    return np.arctan2(y, x)


class PlannerState(object):

    def __init__(self):
        #
        # INPUTS
        #
        # Current pose of the robot (or vehicle)
        self.current_pose = None
        # Glboal waypoints received (i.e. base_waypoints)
        self.global_waypoints = None
        # Closest waypoint
        self.closest_waypoint = None
        self.planned_waypoint = 0
        # Obstacles
        self.obstacles = None
        #
        # OUTPUTS
        #
        # Final (relayed or replanned) trajectory
        self.final_lane = Lane()
        #
        # SIGNALS AND EVENTS
        #
        self.signal_obstacle_detected = False

    def is_initialized(self):
        return self.current_pose is not None \
               and self.global_waypoints is not None \
               and self.closest_waypoint is not None \
               and self.obstacles is not None

    def check_obstacles(self):
        if len(self.obstacles.obstacles) > 0:
            self.signal_obstacle_detected = True
        else:
            self.signal_obstacle_detected = False

    def is_obstacle_detected(self):
        return self.signal_obstacle_detected




class PlannerVisualization(object):

    def __init__(self):
        self.msg_viz_final_trajectory = MarkerArray()
        self.pub_viz_final_trajectory = rospy.Publisher("/final_trajectory/visualization", MarkerArray, queue_size=1)




class ButaPlanner(object):

    def __init__(self, hz, global_frame="map", robot_frame="base_link",
                 max_height = 3.0, prior_seg = 7, poster_seg= 14):
        #
        self.hz = hz
        # Planner parameters
        self.max_height = max_height
        self.prior_seg = prior_seg
        self.poster_seg = poster_seg
        # TF initialization
        self.global_frame = global_frame
        self.robot_frame = robot_frame
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        # Internal state definition
        self.planner_state = PlannerState()
        self.states = {"RELAY": 0, "REPLANNING": 1}
        self.state = self.states["RELAY"]
        # Publisher initialization
        self.pub_final_trajectory = rospy.Publisher("/final_waypoints", Lane, queue_size=1)
        # Subscriber initialization
        # Subscribe the current pose of the vehicle
        self.sub_current_pose = rospy.Subscriber("/current_pose", PoseStamped, self.cb_current_pose)
        # Glboal waypoints
        self.sub_current_lane = rospy.Subscriber("/base_waypoints", Lane, self.cb_lane_waypoints)

        self.sub_closest_waypoint = rospy.Subscriber("/closest_waypoint", Int32, self.cb_closest_waypoints)
        #
        self.sub_obstacles = rospy.Subscriber("/detected_obstacles", DetectedObstacles, self.cb_obstacle_detection)
        #
        # Visualization
        self.planner_visualization = PlannerVisualization()

    def normal_vector_calculation(self, i):
        position_0 = self.planner_state.global_waypoints.waypoints[i].pose.pose.position
        position_1 = self.planner_state.global_waypoints.waypoints[i + 1].pose.pose.position
        norm = np.array([[position_1.x - position_0.x,
                          position_1.y - position_0.y]])
        norm = norm / np.linalg.norm(norm)
        norm = norm[:, [1, 0]]
        return norm

    def tangent_vector_calculation(self, i):
        position_0 = self.planner_state.global_waypoints.waypoints[i].pose.pose.position
        position_1 = self.planner_state.global_waypoints.waypoints[i + 1].pose.pose.position
        tangent = np.array([[position_1.x - position_0.x,
                          position_1.y - position_0.y]])
        return tangent

    # REGION
    # Planner definitions
    #
    def event_plan(self):
        self.planner_state.final_lane.waypoints = []
        # Assign
        wp_beginning = None
        wp_end = None
        for ob in self.planner_state.obstacles.obstacles:
            # STEP #1: Generate trajectory points
            # Middle segment points
            middle_waypoints = []
            for i in range(ob.closest_waypoint - 1, ob.closest_waypoint + 2):
                wp = Waypoint()
                wp.pose = self.planner_state.global_waypoints.waypoints[i].pose
                wp.twist = self.planner_state.global_waypoints.waypoints[i].twist
                position_1 = self.planner_state.global_waypoints.waypoints[i].pose.pose.position
                norm = self.normal_vector_calculation(i)
                position_1.x += norm[0, 0] * self.max_height
                position_1.y += norm[0, 1] * self.max_height
                wp.pose.pose.position = position_1
                middle_waypoints.append(wp)
            # Beginning segments
            beginning_segment_index = ob.closest_waypoint - self.prior_seg
            wp_beginning = self.planner_state.global_waypoints.waypoints[beginning_segment_index]
            v_beginning = np.array([[
                middle_waypoints[0].pose.pose.position.x - wp_beginning.pose.pose.position.x,
                middle_waypoints[0].pose.pose.position.y - wp_beginning.pose.pose.position.y]])
            v_beginning = v_beginning/np.linalg.norm(v_beginning)
            yaw_beginning = np.arctan2(v_beginning[0, 1], v_beginning[0, 0])
            wp_beginning.pose.pose.orientation.x = 0.0
            wp_beginning.pose.pose.orientation.y = 0.0
            wp_beginning.pose.pose.orientation.z = np.sin(yaw_beginning/2.0)
            wp_beginning.pose.pose.orientation.w = np.cos(yaw_beginning/2.0)
            # End segments
            ending_segment_index = ob.closest_waypoint + self.poster_seg
            wp_ending = self.planner_state.global_waypoints.waypoints[ending_segment_index]
            v_ending = np.array([[
                middle_waypoints[-1].pose.pose.position.x - wp_ending.pose.pose.position.x,
                middle_waypoints[-1].pose.pose.position.y - wp_ending.pose.pose.position.y]])
            v_ending = v_ending / np.linalg.norm(v_ending)
            yaw_ending = np.arctan2(v_ending[0, 1], v_ending[0, 0])
            wp_ending.pose.pose.orientation.x = 0.0
            wp_ending.pose.pose.orientation.y = 0.0
            wp_ending.pose.pose.orientation.z = np.sin(yaw_ending / 2.0)
            wp_ending.pose.pose.orientation.w = np.cos(yaw_ending / 2.0)
            # STEP #2: Merge beginning, middle, end parts
            # STEP #2.1: Merge original part
            for i in range(self.planner_state.closest_waypoint, beginning_segment_index):
                self.planner_state.final_lane.waypoints.append(self.planner_state.global_waypoints.waypoints[i])
            # STEP #2.2: Merge replanned part
            for i in range(self.prior_seg):
                wp = Waypoint()
                wp.pose = PoseStamped()
                wp.pose.pose.position.x = wp_beginning.pose.pose.position.x
                wp.pose.pose.position.y = wp_beginning.pose.pose.position.y
                wp.pose.pose.position.x += v_beginning[0, 0] * i
                wp.pose.pose.position.y += v_beginning[0, 1] * i
                wp.pose.pose.orientation.x = wp_beginning.pose.pose.orientation.x
                wp.pose.pose.orientation.y = wp_beginning.pose.pose.orientation.y
                wp.pose.pose.orientation.z = wp_beginning.pose.pose.orientation.z
                wp.pose.pose.orientation.w = wp_beginning.pose.pose.orientation.w
                wp.twist = self.planner_state.global_waypoints.waypoints[i + beginning_segment_index].twist
                self.planner_state.final_lane.waypoints.append(wp)
            for wp in middle_waypoints:
                self.planner_state.final_lane.waypoints.append(wp)
            for i in range(self.poster_seg):
                wp = Waypoint()
                wp.pose = wp_ending.pose
                wp.twist = wp_ending.twist
                wp.pose = PoseStamped()
                wp.pose.pose.position.x = wp_ending.pose.pose.position.x
                wp.pose.pose.position.y = wp_ending.pose.pose.position.y
                wp.pose.pose.orientation.x = wp_ending.pose.pose.orientation.x
                wp.pose.pose.orientation.y = wp_ending.pose.pose.orientation.y
                wp.pose.pose.orientation.z = wp_ending.pose.pose.orientation.z
                wp.pose.pose.orientation.w = wp_ending.pose.pose.orientation.w
                wp.pose.pose.position.x += v_ending[0, 0] * i
                wp.pose.pose.position.y += v_ending[0, 1] * i
                wp.twist = self.planner_state.global_waypoints.waypoints[ob.closest_waypoint + i + 1].twist
                self.planner_state.final_lane.waypoints.append(wp)
        self.planner_visualization.msg_viz_final_trajectory.markers = []
        m = Marker()
        m.action = m.DELETEALL
        self.planner_visualization.msg_viz_final_trajectory.markers.append(m)
        self.planner_visualization.pub_viz_final_trajectory.publish(self.planner_visualization.msg_viz_final_trajectory)
        self.planner_visualization.msg_viz_final_trajectory.markers = []
        for i,wp in enumerate(self.planner_state.final_lane.waypoints):
            m = Marker()
            m.pose = wp.pose.pose
            m.ns = "final_trajectory"
            m.id = i
            m.color.r = 1.0
            m.color.b = 1.0
            m.color.a = 1.0
            m.scale.x = 1.0
            m.scale.y = 1.0
            m.scale.z = 1.0
            m.type = Marker.SPHERE
            m.header.frame_id = "map"
            self.planner_visualization.msg_viz_final_trajectory.markers.append(m)
            m = Marker()
            m.pose = wp.pose.pose
            m.ns = "final_trajectory_orientation"
            m.id = i
            m.color.g = 1.0
            m.color.b = 1.0
            m.color.a = 1.0
            m.scale.x = 0.7
            m.scale.y = 0.2
            m.scale.z = 0.4
            m.type = Marker.ARROW
            m.header.frame_id = "map"
            self.planner_visualization.msg_viz_final_trajectory.markers.append(m)
        if wp_beginning is not None:
            m = Marker()
            m.pose = wp_beginning.pose.pose
            m.ns = "beginning_point"
            m.header.frame_id = "map"
            m.pose = wp_beginning.pose.pose
            m.id = i
            m.color.g = 1.0
            m.color.b = 1.0
            m.color.a = 1.0
            m.scale.x = 1.7
            m.scale.y = 0.2
            m.scale.z = 0.4
            self.planner_visualization.msg_viz_final_trajectory.markers.append(m)
            m = Marker()
            m.pose = wp_beginning.pose.pose
            m.ns = "ending_point"
            m.header.frame_id = "map"
            m.pose = middle_waypoints[-1].pose.pose
            m.id = i
            m.color.g = 1.0
            m.color.b = 1.0
            m.color.a = 1.0
            m.scale.x = 1.7
            m.scale.y = 0.2
            m.scale.z = 0.4
            self.planner_visualization.msg_viz_final_trajectory.markers.append(m)





    # REGION
    # Behavior functions
    def relay(self):
        self.planner_state.final_lane.waypoints = []
        for i in range(self.planner_state.closest_waypoint,
                       len(self.planner_state.global_waypoints.waypoints)):
            self.planner_state.final_lane.waypoints.append(self.planner_state.global_waypoints.waypoints[i])

    # REGION
    # Initialization
    #
    def initialize_timers(self):
        self.timer_pub_lane = rospy.Timer(rospy.Duration(1.0/self.hz), self.cb_plan_timer, reset=True)



    # REGION
    # Timer
    def cb_plan_timer(self, event):
        if self.planner_state.is_initialized():
            if self.state==self.states["RELAY"]:
                print("RELAY")
                if self.planner_state.is_obstacle_detected():
                    self.state = self.states["REPLANNING"]
                    self.event_plan()
                else:
                    self.relay()
            elif self.state==self.states["REPLANNING"]:
                if not self.planner_state.is_obstacle_detected():
                    self.state = self.states["RELAY"]
            self.publish_final_waypoints()
            self.planner_visualization.pub_viz_final_trajectory.publish(self.planner_visualization.msg_viz_final_trajectory)
    # REGION:
    # Subscriber callbacks
    #
    def cb_current_pose(self, data):
        self.planner_state.current_pose = data

    def cb_lane_waypoints(self, data):
        self.planner_state.global_waypoints = data

    def cb_closest_waypoints(self, data):
        self.planner_state.closest_waypoint = data.data

    def cb_obstacle_detection(self, data):
        self.planner_state.obstacles = data
        self.planner_state.check_obstacles()

    # REGION:
    # Publish messages
    #
    def publish_final_waypoints(self):
        self.pub_final_trajectory.publish(self.planner_state.final_lane)


def main():
    rospy.init_node("buta_planner")
    buta_planner = ButaPlanner(20, max_height=2.0)
    buta_planner.initialize_timers()
    rospy.spin()

if __name__ == "__main__":
    main()