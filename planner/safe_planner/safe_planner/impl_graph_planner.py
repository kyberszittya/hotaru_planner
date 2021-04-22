import math

from queue import PriorityQueue


from safe_planner.definition_graph_planner import AbstractGraphAlgorithmPlannerComponent
from safe_planner.util_math import euclidean_distance_2d


class AStarPlannerComponent(AbstractGraphAlgorithmPlannerComponent):
    def __init__(self, env_repr, goal_epsilon, idx_start=0, idx_end=-1,
                 distance_metric=euclidean_distance_2d):
        AbstractGraphAlgorithmPlannerComponent.__init__(self, env_repr, goal_epsilon,
                                                        idx_start=idx_start,
                                                        idx_end=idx_end,
                                                        distance_metric=distance_metric)

    def create_fringe(self):
        return PriorityQueue()

    def heuristic(self, p):
        return self.distance_metric((p[0], p[1]), (self.goal.pose.position.x, self.goal.pose.position.y))

    def calc_expansion_cost(self, state):
        p = state[0], state[1]
        return self.heuristic(p) + self.environment_snapshot.get_cost(p)










