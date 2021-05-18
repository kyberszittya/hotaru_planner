import rclpy
from rclpy.node import Node

from hotaru_msgs.msg import Trajectory, Waypoint
from definition_graph_planner import AbstractPlannerImplementation

import time

class PlannerNode(Node):
    def __init__(self, name, planning_rate: float, planner: AbstractPlannerImplementation):
        self.name = name
        Node.__init__(self, "planner_"+name)
        # Abstract implementation of planner (adapter)
        self.planner_adapter = planner
        # Basic I/O interface (input and refined trajectory)
        # Subscriber interface for trajectory planning
        self._refined_trajectory = self.create_publisher(Trajectory, '/'.join([self.name, 'refined_trajectory']), 1)
        self._input_trajectory = None
        # Planner timer cycle
        self._plan_timer = self.create_timer(planning_rate, self.cb_timer_planner)
        self.get_logger().info("Starting planner node {0}".format(self.name))

    def connect_to_node(self, input_node):
        self._input_trajectory = self.create_subscription(Trajectory, '/'.join([input_node.name, 'refined_trajectory']),
                                                          self.cb_trajectory, 1)

    def cb_trajectory(self, msg: Trajectory):
        self.planner_adapter.on_update_trajectory(msg)

    def cb_timer_planner(self):
        t0_astar_plan = time.perf_counter()
        self.planner_adapter.refine()
        self.planner_adapter.serialize_trajectory()
        self._refined_trajectory.publish(self.planner_adapter.get_trajectory())
        t1_astar_plan = time.perf_counter()
        self.get_logger().info('Time to plan (A*): {0}'.format(t1_astar_plan - t0_astar_plan))
