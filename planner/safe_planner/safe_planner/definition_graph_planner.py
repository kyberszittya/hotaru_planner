from safe_planner.definition_planner_components import AbstractPlannerImplementation

from safe_planner.util_math import euclidean_distance_2d

from queue import PriorityQueue


class AbstractGraphAlgorithmPlannerComponent(AbstractPlannerImplementation):
    def __init__(self, env_repr, goal_epsilon,
                 idx_start=0, idx_end=-1, distance_metric=euclidean_distance_2d,
                 bypass_planning=False):
        AbstractPlannerImplementation.__init__(self, env_repr, idx_start, idx_end, bypass_planning=bypass_planning)
        self.distance_metric = distance_metric
        self.goal_epsilon = goal_epsilon

    @staticmethod
    def backtrack_trajectory(node):
        n = node
        trajectory = []
        while n.parent is not None:
            n = n.parent
            trajectory.append(n.desc_coord)
        print(trajectory)
        return trajectory

    def is_goal(self, p, g):
        return self.distance_metric(p, g) < self.goal_epsilon

    def calc_expansion_cost(self, state):
        raise NotImplementedError

    def create_fringe(self):
        raise NotImplementedError

    def plan_algorithm(self):
        visited = set()
        fringe = self.create_fringe()
        px, py, pz = self.start[0], self.start[1], 0.0
        gx, gy, gz = self.goal[0], self.goal[1], 0.0
        first_node = self.environment_snapshot.get_state_repr((px, py))
        visited.add(first_node.coord)
        fringe.put((first_node.value, first_node))
        while not fringe.empty():
            value, next_node = fringe.get()
            px, py = next_node.desc_coord
            if self.is_goal((px, py), (gx, gy)):
                return self.backtrack_trajectory(next_node), next_node
            for n in self.environment_snapshot.expand_strategy(next_node):
                if n.coord not in visited:
                    n.value = self.calc_expansion_cost(n.desc_coord)
                    fringe.put((n.value, n))
                    visited.add(n.coord)
