import rospy

from rei_monitoring_msgs.msg import  DetectedObstacles, Obstacle
from std_msgs.msg import Int32

import numpy as np

import tf2_ros

from autoware_msgs.msg import Lane, DetectedObjectArray

from geometry_msgs.msg import PointStamped, Point, PoseStamped

from visualization_msgs.msg import MarkerArray, Marker

import tf2_geometry_msgs


def linear_distance(v0x, v0y, v1x, v1y, px, py):
    nom = np.abs((v1x - v0x) * (v1y - py) - (v0x - px) * (v1y - v0y))
    nz = np.sqrt((v1y - v0y) ** 2 + (v1x - v0x) ** 2)
    return nom / nz

def signed_linear_distance(v0x, v0y, v1x, v1y, px, py):
    nom = (v1x - v0x) * (v1y - py) - (v0x - px) * (v1y - v0y)
    nz = np.sqrt((v1y - v0y) ** 2 + (v1x - v0x) ** 2)
    return -nom / nz


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
        return self.global_waypoints is not None and self.obstacles is not None and self.current_pose is not None


class ObstacleProposal(object):
    # [o, d_lin, object_closest_waypoint, position_obstacle_global, 1,
    # mind, position_obstacle_base, 0.0, 0.0]

    def __init__(self, o, d_lin, object_closest_waypoint, position_obstacle_global,
                 ttl, d_min, position_obstacle_base, d_left, d_right):
        self.obstacle = o
        self.d_linear = d_lin
        self.object_closest_waypoint = object_closest_waypoint
        self.position_obstacle_global = position_obstacle_global
        self.ttl = ttl
        self.d_min = d_min
        self.position_obstacle_base = position_obstacle_base
        self.d_left = d_left
        self.d_right = d_right


class ObstacleDetectorVisualization(object):

    def __init__(self, global_frame, robot_frame, robot_radius):
        self.pub_viz_detected_obstacle_marker = rospy.Publisher("/detected_obstacles/visualization", MarkerArray,
                                                                queue_size=1)
        self.pub_viz_vehicle_marker = rospy.Publisher("/vehicle_visualization", MarkerArray,
                                                      queue_size=1)
        self.marker_obstacles = MarkerArray()
        self.robot_radius = robot_radius
        self.marker_robot_visualization = MarkerArray()
        self.robot_frame = robot_frame
        self.global_frame = global_frame

    def publish_detected_object_marker(self, obstacles):
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        self.marker_obstacles.markers.append(delete_marker)
        self.pub_viz_detected_obstacle_marker.publish(self.marker_obstacles)
        for i,o in enumerate(obstacles):
            m = Marker()
            p = Point()
            p.x = o[0]
            p.y = o[1]
            m.type = Marker.SPHERE
            m.scale.x = 1.0
            m.scale.y = 1.0
            m.scale.z = 1.0
            m.color.r = 1.0
            m.color.g = 0.0
            m.color.b = 0.0
            m.color.a = 1.0
            m.header.frame_id = self.robot_frame
            m.pose.position = p
            m.id = i
            m.ns = "obstacle"
            self.marker_obstacles.markers.append(m)
        self.pub_viz_detected_obstacle_marker.publish(self.marker_obstacles)

    def publish_robot_safety_radius(self):
        self.marker_robot_visualization.markers = []
        marker_robot_radius = Marker()
        marker_robot_radius.header.frame_id = "base_link"
        marker_robot_radius.type = Marker.CYLINDER
        marker_robot_radius.scale.x = 2*self.robot_radius
        marker_robot_radius.scale.y = 2*self.robot_radius
        marker_robot_radius.scale.z = 0.1
        marker_robot_radius.color.r = 1.0
        marker_robot_radius.color.g = 0.5
        marker_robot_radius.color.a = 1.0
        marker_robot_radius.ns = "robot_radius"
        marker_robot_radius.id = 1
        self.marker_robot_visualization.markers.append(marker_robot_radius)
        self.pub_viz_vehicle_marker.publish(self.marker_robot_visualization)


class ConvexObjectObstacleMonitor(object):

    def __init__(self, hz, safe_threshold=1.1,
                 road_width = 6.0,
                 far_clip = 25,
                 global_frame="map", robot_frame="base_link", visualize=True):
        self.hz = hz
        self.road_width = road_width
        self.far_clip = far_clip
        self.visualize = visualize
        self.safe_threshold = safe_threshold
        # TF
        self.global_frame = global_frame
        self.robot_frame = robot_frame
        self.sensor_frame = ""
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        #
        # Internal state
        #
        self.obstacle_monitor_state = ObstacleDetectorState()
        #
        # Subscribers
        #
        self.sub_reference_waypoint = rospy.Subscriber("base_waypoints", Lane, self.cb_base_waypoints)
        self.sub_closest_waypoint = rospy.Subscriber("/closest_waypoint", Int32, self.cb_closest_waypoint)
        self.sub_detected_obstacles = rospy.Subscriber("detection/lidar_detector/objects", DetectedObjectArray,
                                                       self.cb_detected_objects)
        self.sub_current_pose = rospy.Subscriber("/current_pose", PoseStamped, self.cb_current_pose)
        #
        # Publishers
        #
        self.pub_detected_obstacles = rospy.Publisher("detected_obstacles", DetectedObstacles, queue_size=1)
        #
        # Visualization
        #
        if self.visualize:
            self.visualization_detector = ObstacleDetectorVisualization(self.global_frame,
                                                                        self.robot_frame, self.safe_threshold)


    def initialize_timers(self):
        self.timer_detection = rospy.Timer(rospy.Duration(1.0/self.hz), self.cb_timer_obstacle, reset=True)


    # REGION
    # Subscriber callbacks
    def cb_base_waypoints(self, data):
        self.obstacle_monitor_state.global_waypoints = data

    def cb_closest_waypoint(self, data):
        self.obstacle_monitor_state.closest_waypoint = data.data

    def cb_current_pose(self, data):
        self.obstacle_monitor_state.current_pose = data


    def cb_detected_objects(self, data):
        self.obstacle_monitor_state.obstacles = data
        self.sensor_frame = data.header.frame_id

    def is_initialized(self):
        return len(self.sensor_frame) > 0 and self.obstacle_monitor_state.is_initialized()

    def is_lane_valid(self):
        return len(self.obstacle_monitor_state.global_waypoints.waypoints) > 3

    # REGION
    # Timer callbacks
    def cb_timer_obstacle(self, event):
        if self.is_initialized() and self.is_lane_valid():
            try:
                # TF
                self.trans_sensor_global = self.tf_buffer.lookup_transform(self.global_frame, self.sensor_frame, rospy.Time())
                mon_state = self.obstacle_monitor_state
                alive_obstacles = set()

                for o in self.obstacle_monitor_state.obstacles.objects:
                    # To perform transformation, we define a stamped type
                    o_point = PointStamped()
                    o_point.point.x = o.pose.position.x
                    o_point.point.y = o.pose.position.y
                    o_point.point.z = o.pose.position.z
                    position_obstacle_base = tf2_geometry_msgs.do_transform_point(
                        o_point, self.trans_sensor_base).point
                    raw_pose = PointStamped()
                    raw_pose.header.frame_id = self.sensor_frame
                    raw_pose.point.x = o.pose.position.x
                    raw_pose.point.y = o.pose.position.y
                    raw_pose.point.z = o.pose.position.z
                    position_obstacle_global = tf2_geometry_msgs.do_transform_point(
                        raw_pose, self.trans_sensor_global).point
                    # STEP 0: Check if object has been already detected
                    match = False
                    for i,ob in enumerate(self.obstacle_monitor_state.detected_obstacles):
                        if (ob.position_obstacle_global.x - position_obstacle_global.x) < 0.5 and \
                                (ob.position_obstacle_global.y - position_obstacle_global.y) < 0.5:
                            match = True
                            alive_obstacles.add(i)
                            ob.position_obstacle_global = position_obstacle_global
                            ob.ttl += 1
                            if ob.ttl > int(self.hz)*2:
                                ob.ttl = int(self.hz*2)
                            ob.position_obstacle_base = position_obstacle_base
                    if not match:
                        # STEP 1: Search for the closest waypoint of the center
                        mini, mind = mon_state.closest_waypoint, np.inf
                        # TODO: check this
                        for i in range(mon_state.closest_waypoint,
                                       len(mon_state.global_waypoints.waypoints) - 1):
                            # Object that are way far from a waypoint are not of interest
                            dx = mon_state.global_waypoints.waypoints[i].pose.pose.position.x - position_obstacle_global.x
                            dy = mon_state.global_waypoints.waypoints[i].pose.pose.position.y - position_obstacle_global.y
                            d = np.sqrt(dx ** 2 + dy ** 2)
                            if d < 2.0:
                                if mind > d:
                                    mini, mind = i, d
                                    #detected_obstacles.append((o, d, object_closest_waypoint, position_obstacle_global))
                        object_closest_waypoint = mini

                        d_lin = linear_distance(
                            mon_state.global_waypoints.waypoints[mini].pose.pose.position.x,
                            mon_state.global_waypoints.waypoints[mini].pose.pose.position.y,
                            mon_state.global_waypoints.waypoints[mini + 1].pose.pose.position.x,
                            mon_state.global_waypoints.waypoints[mini + 1].pose.pose.position.y,
                            position_obstacle_global.x,
                            position_obstacle_global.y
                        )
                        if d_lin < self.safe_threshold:
                            self.obstacle_monitor_state.detected_obstacles.append(
                                #[o, d_lin, object_closest_waypoint, position_obstacle_global, 1,
                                # mind, position_obstacle_base, 0.0, 0.0]
                                ObstacleProposal(o, d_lin, object_closest_waypoint,
                                                 position_obstacle_global,
                                                 1, mind, position_obstacle_base, 0.0, 0.0)
                            )

                # Visualization TODO: migrate to visualization component
                marker_visualization = MarkerArray()
                delete_marker = Marker()
                delete_marker.action = Marker.DELETEALL
                marker_visualization.markers.append(delete_marker)
                self.visualization_detector.pub_viz_detected_obstacle_marker.publish(marker_visualization)
                marker_visualization.markers = []
                # STEP 1.1: Set alive obstacles
                for o in self.obstacle_monitor_state.detected_obstacles:
                    if o.ttl > self.hz:
                        match = False
                        for alive_ob in self.obstacle_monitor_state.alive_obstacles:
                            #if alive_ob[3].x - o[3].x < 0.5 and alive_ob[3].y - o[3].y < 0.5:
                            if alive_ob.position_obstacle_global.x - o.position_obstacle_global.x < 0.5 \
                                    and alive_ob.position_obstacle_global.y - o.position_obstacle_global.y < 0.5:
                                # Update
                                alive_ob.position_obstacle_global = o.position_obstacle_global
                                alive_ob.object_closest_waypoint = o.object_closest_waypoint
                                #
                                match = True
                        if not match:
                            self.obstacle_monitor_state.alive_obstacles.append(o)
                # STEP 2: get distance of points
                for i,o in enumerate(self.obstacle_monitor_state.alive_obstacles):
                    leftmost_point_distance = np.inf
                    rightmost_point_distance = -np.inf
                    object_closest_waypoint = o.object_closest_waypoint
                    # For visualization purposes
                    left_point = None
                    right_point = None
                    #
                    for cp in o.obstacle.convex_hull.polygon.points:
                        raw_point_position = PointStamped()
                        raw_point_position.point.x = cp.x
                        raw_point_position.point.y = cp.y
                        position_hull_point = tf2_geometry_msgs.do_transform_point(
                            raw_point_position, self.trans_sensor_global).point
                        d = signed_linear_distance(
                            mon_state.global_waypoints.waypoints[object_closest_waypoint].pose.pose.position.x,
                            mon_state.global_waypoints.waypoints[object_closest_waypoint].pose.pose.position.y,
                            mon_state.global_waypoints.waypoints[object_closest_waypoint + 1].pose.pose.position.x,
                            mon_state.global_waypoints.waypoints[object_closest_waypoint + 1].pose.pose.position.y,
                            position_hull_point.x,
                            position_hull_point.y
                        )
                        if leftmost_point_distance > d:
                            leftmost_point_distance = d
                            left_point = position_hull_point
                        if rightmost_point_distance < d:
                            rightmost_point_distance = d
                            right_point = position_hull_point
                    o.d_left = leftmost_point_distance
                    o.d_right = rightmost_point_distance
                    #
                    # VISUALIZATION
                    #
                    if self.visualize:
                        m = Marker()
                        m.header.frame_id = self.global_frame
                        m.pose.orientation.w = 1.0
                        m.ns = "obstacle"
                        m.pose.position = o.position_obstacle_global
                        m.type = Marker.SPHERE
                        m.color.r = 1.0
                        m.color.a = 1.0
                        m.scale.x = 1.0
                        m.scale.y = 1.0
                        m.scale.z = 1.0
                        m.id = i
                        marker_visualization.markers.append(m)
                        #
                        """
                        m_right_point = Marker()
                        m_right_point.pose.orientation.w = 1.0
                        m_right_point.type = Marker.SPHERE
                        m_right_point.header.frame_id = self.global_frame
                        m_right_point.ns = "right_points"
                        m_right_point.pose.position = right_point
                        m_right_point.color.r = 1.0
                        m_right_point.color.a = 1.0
                        m_right_point.scale.x = 0.5
                        m_right_point.scale.y = 0.5
                        m_right_point.scale.z = 0.5
                        m_right_point.id = i
                        marker_visualization.markers.append(m_right_point)
                        # Visualize borders
                        m_left_point = Marker()
                        m_left_point.pose.orientation.w = 1.0
                        m_left_point.header.frame_id = self.global_frame
                        m_left_point.ns = "left_points"
                        m_left_point.type = Marker.SPHERE
                        m_left_point.pose.position = left_point
                        m_left_point.color.r = 1.0
                        m_left_point.color.a = 1.0
                        m_left_point.scale.x = 0.5
                        m_left_point.scale.y = 0.5
                        m_left_point.scale.z = 0.5
                        m_left_point.id = i
                        marker_visualization.markers.append(m_left_point)
                        """
                        # Visualize closest waypoint
                        m_closest_waypoint = Marker()
                        m_closest_waypoint.pose.orientation.w = 1.0
                        m_closest_waypoint.header.frame_id = self.global_frame
                        m_closest_waypoint.ns = "closest_waypoint_1"
                        m_closest_waypoint.type = Marker.SPHERE
                        m_closest_waypoint.pose.position = self.obstacle_monitor_state.global_waypoints.waypoints[
                            o.object_closest_waypoint].pose.pose.position
                        m_closest_waypoint.color.r = 1.0
                        m_closest_waypoint.color.g = 0.7
                        m_closest_waypoint.color.a = 1.0
                        m_closest_waypoint.scale.x = 0.7
                        m_closest_waypoint.scale.y = 0.7
                        m_closest_waypoint.scale.z = 0.7
                        m_closest_waypoint.id = i
                        marker_visualization.markers.append(m_closest_waypoint)
                        # TODO: waypoint
                        m_closest_waypoint = Marker()
                        m_closest_waypoint.pose.orientation.w = 1.0
                        m_closest_waypoint.header.frame_id = self.global_frame
                        m_closest_waypoint.ns = "closest_waypoint_next"
                        m_closest_waypoint.type = Marker.SPHERE
                        m_closest_waypoint.pose.position = self.obstacle_monitor_state.global_waypoints.waypoints[
                            o.object_closest_waypoint + 1].pose.pose.position
                        m_closest_waypoint.color.r = 1.0
                        m_closest_waypoint.color.g = 0.7
                        m_closest_waypoint.color.a = 1.0
                        m_closest_waypoint.scale.x = 0.7
                        m_closest_waypoint.scale.y = 0.7
                        m_closest_waypoint.scale.z = 0.7
                        m_closest_waypoint.id = i
                        marker_visualization.markers.append(m_closest_waypoint)
                        # previous waypoint
                        m_closest_waypoint = Marker()
                        m_closest_waypoint.pose.orientation.w = 1.0
                        m_closest_waypoint.header.frame_id = self.global_frame
                        m_closest_waypoint.ns = "closest_waypoint_prev"
                        m_closest_waypoint.type = Marker.SPHERE
                        m_closest_waypoint.pose.position = self.obstacle_monitor_state.global_waypoints.waypoints[
                            o.object_closest_waypoint - 1].pose.pose.position
                        m_closest_waypoint.color.r = 1.0
                        m_closest_waypoint.color.g = 0.7
                        m_closest_waypoint.color.a = 1.0
                        m_closest_waypoint.scale.x = 0.7
                        m_closest_waypoint.scale.y = 0.7
                        m_closest_waypoint.scale.z = 0.7
                        m_closest_waypoint.id = i
                        marker_visualization.markers.append(m_closest_waypoint)
                self.visualization_detector.pub_viz_detected_obstacle_marker.publish(marker_visualization)
                self.visualization_detector.publish_robot_safety_radius()
                # STEP 3: manage obstacle list (decay, remove old)
                for i,o in enumerate(self.obstacle_monitor_state.detected_obstacles):
                    if o.ttl <= 0:
                        self.obstacle_monitor_state.detected_obstacles.remove(o)
                    elif i not in alive_obstacles:
                        o.ttl -= 1
                # STEP 4: publish alive objects
                # STEP 5: check alive objects
                for o in self.obstacle_monitor_state.alive_obstacles:
                    print(o.position_obstacle_base.x, o.position_obstacle_base.y, o.d_min)
                    if self.obstacle_monitor_state.closest_waypoint > o.object_closest_waypoint + 5:
                        self.obstacle_monitor_state.alive_obstacles.remove(o)
                print("----")
                # STEP 6: publish alive objects
                self.obstacle_monitor_state.msg_alive_obstacles.obstacles = []
                for o in self.obstacle_monitor_state.alive_obstacles:
                    # Setup message
                    msg_ob = Obstacle()
                    msg_ob.closest_waypoint = o.object_closest_waypoint
                    msg_ob.pose.position = o.position_obstacle_base
                    msg_ob.global_pose.position = o.position_obstacle_global
                    msg_ob.obstacle_type = Obstacle.STATIC_OBSTACLE
                    msg_ob.linear_distance = o.d_min
                    msg_ob.leftmost_distance = o.d_left
                    msg_ob.rightmost_distance = o.d_right
                    self.obstacle_monitor_state.msg_alive_obstacles.obstacles.append(msg_ob)
                # Publish
                self.pub_detected_obstacles.publish(self.obstacle_monitor_state.msg_alive_obstacles)

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logerr("TF error")
            except IndexError:
                rospy.logerr("End or beinning of line")
                self.obstacle_monitor_state.detected_obstacles = []
                self.obstacle_monitor_state.alive_obstacles = []


def main():
    rospy.init_node("obstacle_convex_detection")
    obstacle_monitor = ConvexObjectObstacleMonitor(10, road_width=3, safe_threshold=1.3)
    obstacle_monitor.initialize_timers()
    rospy.spin()

if __name__=="__main__":
    main()