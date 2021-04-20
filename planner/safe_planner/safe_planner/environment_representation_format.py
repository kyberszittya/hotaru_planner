import rclpy
from rclpy.node import Node

import numpy as np


class AbstractEnvironmentRepresentation(object):

    def __init__(self):
        self.data = None

    def update_representation(self, data):
        self.data = data

    def expand_strategy(self, state):
        raise NotImplementedError


class StateTreeNode(object):

    def __init__(self, coord):
        self.coord


class Grid(object):
    def __init__(self, X, Y, values):
        self.X = X
        self.Y = Y
        self.values = values
        # Calculate dimensions
        self.dim_x = self.values.shape[0]
        self.dim_y = self.values.shape[1]
        # Calculate extreme points
        self.min_x = np.min(self.X)
        self.max_x = np.max(self.X)
        self.min_y = np.min(self.Y)
        self.max_y = np.max(self.Y)
        # Calculate steps
        self.step_x = (self.max_x - self.min_x) / self.dim_x
        self.step_y = (self.max_y - self.min_y) / self.dim_y

    def coord_to_index(self, coord):
        grid_coord = int(coord[0] * self.step_x - self.min_x), \
                     int(coord[1] * self.step_y - self.min_y)
        if self.min_x <= coord[0] <= self.max_x and self.min_y <= coord[1] <= self.max_y:
            return grid_coord
        return None


class GridBasedEnvironmentRepresentation(AbstractEnvironmentRepresentation):

    def __init__(self, grid):
        AbstractEnvironmentRepresentation.__init__(self)
        self.grid = grid

    def expand_strategy(self, state):
        # Expand all nodes in neighbors
        x, y = state.coord
        neighbors = []
        indices_x = [x - self.grid.step_x, x, x + self.grid.step_x]
        indices_y = [y - self.grid.step_y, y, y + self.grid.step_y]
        for ix in indices_x:
            for iy in indices_y:
                n = self.grid.coord_to_index((ix, iy))
                if n is not None:
                    neighbors += [n]
        return neighbors
