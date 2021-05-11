import math

from queue import PriorityQueue


from safe_planner.definition_graph_planner import AbstractGraphAlgorithmPlannerComponent
from safe_planner.util_math import euclidean_distance_2d


class AStarPlannerComponent(AbstractGraphAlgorithmPlannerComponent):
    def __init__(self, env_repr, goal_epsilon, idx_start=0, idx_end=-1,
                 distance_metric=euclidean_distance_2d,
                 bypass_planning=False):
        AbstractGraphAlgorithmPlannerComponent.__init__(self, env_repr, goal_epsilon,
                                                        idx_start=idx_start,
                                                        idx_end=idx_end,
                                                        distance_metric=distance_metric,
                                                        bypass_planning=bypass_planning)

    def create_fringe(self):
        return PriorityQueue()

    def heuristic(self, p):
        return self.distance_metric((p[0], p[1]), (self.goal[0], self.goal[1]))

    def calc_expansion_cost(self, state):
        p = state[0], state[1]
        return self.heuristic(p) + self.environment_snapshot.get_cost(p)

    """
    Act on trajectory update, in this case replan from extreme points
    
    Algorithm:
    0. Report if got an empty input trajectory -> if traj. is empty, do not proceed 
    1. Search for p0 starting point (first extreme point), p0 != null
    1.1.s If the state is not on the environment representation snapshot use this point as starting point
    1.1.f Otherwise, use state as starting point
    2. Search for p1 ending point (second extreme point),  p1 == null
    
    TODO: act on separating strategy into another component
    """
    def on_update_trajectory(self, traj):
        # Step 0.
        if len(traj.waypoints) > 0:
            # Step 1.
            p0 = None
            i_p0 = 0
            for i, wp in enumerate(traj.waypoints):
                if self.environment_snapshot.is_object_inclusion(
                        (wp.pose.pose.position.x, wp.pose.pose.position.y)) is not None:
                    # Starting point found!
                    p0 = wp.pose.pose.position
                    i_p0 = i
                    break
            # Check if state inside env. representation
            self.start = p0.x, p0.y, 0.0
            if self.state is not None:
                if self.environment_snapshot.is_object_inclusion((
                        self.state.pose.pose.position.x, self.state.pose.pose.position.y)):
                    self.start = self.state.pose.position.x, self.state.pose.position.y, 0.0
            # Step 2.
            i_p1 = i_p0
            for i, wp in enumerate(traj.waypoints[i_p0:]):
                p1 = traj.waypoints[i - 1].pose.pose.position
                if self.environment_snapshot.is_object_inclusion(
                        (wp.pose.pose.position.x, wp.pose.pose.position.y)) is None:
                    # Starting point found!
                    i_p1 = i - 1
                    break
            self.goal = p1.x, p1.y, 0.0
        else:
            # TODO: interactive reporting
            print("Empty trajectory")

