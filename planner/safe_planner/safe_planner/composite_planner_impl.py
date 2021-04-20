import rclpy
import math
from rclpy.node import Node

from queue import PriorityQueue

import numpy as np

import matplotlib.pyplot as plt

from hotaru_msgs.msg import Trajectory, Waypoint
from safe_planner.environment_representation_format import AbstractEnvironmentRepresentation, StateTreeNode


class AbstractPlannerImplementation(object):

    def __init__(self, env_repr, idx_start=0, idx_end=-1):
        self.refined_trajectory = Trajectory()
        self.trajectory_points = []
        self.reference_trajectory = Trajectory()
        self.environment_snapshot = env_repr
        self.idx_start = idx_start
        self.idx_end = idx_end
        self.goal = None
        self.state = None

    def update_input_trajectory(self, traj):
        self.refined_trajectory = traj

    def refine(self):
        if self.goal is not None and self.state is not None:
            self.trajectory_points, goal_node = self.plan_algorithm()
            self.construct_trajectory()

    def set_goal(self, goal):
        self.goal = goal

    def set_state(self, state):
        self.state = state

    def get_trajectory_array(self):
        return np.array(self.trajectory_points)

    def construct_trajectory(self):
        for p in self.trajectory_points:
            wp = Waypoint()
            wp.pose.pose.position.x = p[0]
            wp.pose.pose.position.y = p[1]
            self.refined_trajectory.waypoints.append(wp)

    def plan(self, state, goal):
        self.set_state(state)
        self.set_goal(goal)
        self.refine()

    def plan_algorithm(self):
        raise NotImplementedError

    def get_trajectory(self):
        return self.refined_trajectory


def euclidean_distance_2d(state, goal):
    dx = state[0] - goal[0]
    dy = state[1] - goal[1]
    return math.sqrt(dx**2 + dy**2)


class AStarPlannerImplementation(AbstractPlannerImplementation):

    def __init__(self, env_repr, goal_epsilon, idx_start=0, idx_end=-1):
        AbstractPlannerImplementation.__init__(self, env_repr, idx_start, idx_end)
        self.goal_epsilon = goal_epsilon

    def heuristic(self, p):
        return euclidean_distance_2d((p[0], p[1]), (self.goal.pose.position.x, self.goal.pose.position.y))

    def calc_expansion_cost(self, state):
        p = state[0], state[1]
        return self.heuristic(p) + self.environment_snapshot.get_cost(p)

    def is_goal(self, p, g):
        return euclidean_distance_2d(p, g) < self.goal_epsilon

    @staticmethod
    def backtrack_trajectory(node):
        n = node
        trajectory = []
        while n.parent is not None:
            n = n.parent
            trajectory.append(n.desc_coord)
        return trajectory

    def plan_algorithm(self):
        visited = set()
        fringe = PriorityQueue()
        px, py, pz = self.state.pose.position.x, self.state.pose.position.y, self.state.pose.position.z
        first_node = StateTreeNode(self.environment_snapshot.grid.coord_to_index((px, py)),
                                   (px, py), self.calc_expansion_cost((px, py)))
        visited.add(first_node.coord)
        fringe.put((first_node.value, first_node))
        while not fringe.empty():
            value, next_node = fringe.get()
            px, py = next_node.desc_coord
            if self.is_goal((px, py), (self.goal.pose.position.x, self.goal.pose.position.y)):
                return self.backtrack_trajectory(next_node), next_node
            for n in self.environment_snapshot.expand_strategy(next_node):
                if n.coord not in visited:
                    n.value = self.calc_expansion_cost(n.desc_coord)
                    fringe.put((n.value, n))
                    visited.add(n.coord)


class PlannerNode(Node):
    def __init__(self, name, planning_rate, planner):
        rclpy.Node.__init__(self, "planner_"+name)
        # Abstract implementation of planner (adapter)
        self.planner_adapter = planner
        # Basic I/O interface (input and refined trajectory)
        # Subscriber interface for trajectory planning
        self._refined_trajectory = self.create_publisher(Trajectory, 'refined_trajectory')
        self._input_trajectory = self.create_subscription(Trajectory, 'input_trajectory', self.cb_trajectory)
        # Planner timer cycle
        self._plan_timer = self.create_timer(planning_rate, self.cb_timer_planner)

    def cb_trajectory(self, msg):
        self.planner_adapter.update_trajectory(msg)

    def cb_timer_planner(self):
        self.planner_adapter.refine()
        self.planner_adapter.construct_trajectory()
        self._refined_trajectory.publish(self.planner_adapter.get_trajectory())



