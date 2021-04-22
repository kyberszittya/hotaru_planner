import rclpy
from rclpy.node import Node

from hotaru_msgs.msg import Trajectory, Waypoint


class PlannerNode(Node):
    def __init__(self, name, planning_rate, planner):
        self.name = name
        Node.__init__(self, "planner_"+name, namespace=name)
        # Abstract implementation of planner (adapter)
        self.planner_adapter = planner
        # Basic I/O interface (input and refined trajectory)
        # Subscriber interface for trajectory planning
        self._refined_trajectory = self.create_publisher(Trajectory, 'refined_trajectory', 1)
        self._input_trajectory = self.create_subscription(Trajectory, 'input_trajectory', self.cb_trajectory, 1)
        # Planner timer cycle
        self._plan_timer = self.create_timer(planning_rate, self.cb_timer_planner)
        self.get_logger().info("Starting planner node {0}".format(self.name))

    def cb_trajectory(self, msg):
        self.planner_adapter.update_trajectory(msg)

    def cb_timer_planner(self):
        self.planner_adapter.refine()
        self.planner_adapter.construct_trajectory()
        self._refined_trajectory.publish(self.planner_adapter.get_trajectory())