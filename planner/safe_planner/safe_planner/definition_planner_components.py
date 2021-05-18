from geometry_msgs.msg import PoseStamped
from hotaru_msgs.msg import Trajectory, Waypoint

import numpy as np

class AbstractPlannerImplementation(object):

    def __init__(self, env_repr, idx_start=0, idx_end=-1, bypass_planning=False):
        # TODO: solve with states
        self.bypass_planning = bypass_planning
        #
        self.refined_trajectory = Trajectory()
        self.trajectory_points = []
        self.reference_trajectory = Trajectory()
        self.environment_snapshot = env_repr
        self.idx_start = idx_start
        self.idx_end = idx_end
        self.goal = None
        self.start = None
        self.state = None

    def update_input_trajectory(self, traj):
        self.refined_trajectory = traj

    def refine(self):
        if self.goal is not None and self.start is not None and not self.bypass_planning:
            self.trajectory_points, goal_node = self.plan_algorithm()
            #self.construct_trajectory()

    def set_goal(self, goal):
        if isinstance(goal, PoseStamped):
            self.goal = goal.pose.position.x, goal.pose.position.y, goal.pose.position.z
        else:
            self.goal = goal

    def set_state(self, state):
        self.state = state

    def set_start_point(self, start):
        if isinstance(start, PoseStamped):
            self.start = start.pose.position.x, start.pose.position.y, start.pose.position.z
        else:
            self.start = start

    """
    Act on trajectory update 
    """
    def on_update_trajectory(self, traj) -> None:
        raise NotImplementedError

    def plan_algorithm(self):
        raise NotImplementedError

    def serialize_trajectory(self):
        self.refined_trajectory.waypoints.clear()
        for p in self.trajectory_points:
            wp = Waypoint()
            wp.pose.pose.position.x = p[0]
            wp.pose.pose.position.y = p[1]
            self.refined_trajectory.waypoints.append(wp)

    def plan(self, state, goal):
        self.set_state(state)
        self.set_goal(goal)
        self.refine()

    def get_trajectory_array(self):
        return np.array(self.trajectory_points)

    def get_trajectory(self):
        return self.refined_trajectory