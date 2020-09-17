import numpy as np

class SearchNode(object):

    def __init__(self, x, y, cost, parent_index):
        self.x = x
        self.y = y
        self.cost = cost
        self.parent_index = parent_index

class AstarPlanner(object):

    def __init__(self):
        pass

    def set_configuration(self, start, goal):
        self.start = start
        self.goal = goal

    def set_grid(self, grid, resolution_x, resolution_y):
        self.grid = grid
        self.resolution_x = resolution_x
        self.resolution_y = resolution_y

    def plan(self):
        start_node = SearchNode(self.start[0], self.start[1], None)
        goal_node = SearchNode(self.goal[0], self.goal[1], None)
        open


