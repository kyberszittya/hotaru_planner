from hotaru_msgs.msg import Trajectory, Waypoint

import numpy as np

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