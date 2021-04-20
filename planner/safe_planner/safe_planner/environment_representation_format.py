import rclpy
from rclpy.node import Node

import numpy as np
import math


class AbstractEnvironmentRepresentation(object):
    def __init__(self):
        self.data = None

    def update_representation(self, data):
        self.data = data

    def expand_strategy(self, state):
        raise NotImplementedError

    def get_cost(self, coord):
        raise NotImplementedError


class StateTreeNode(object):
    def __init__(self, coord, desc_coord, value, parent=None):
        self.coord = coord
        self.desc_coord = desc_coord
        self.value = value
        self.parent = parent

    def __le__(self, other):
        return self.value <= other.value

    def __lt__(self, other):
        return self.value < other.value

    def __ge__(self, other):
        return self.value >= other.value

    def __gt__(self, other):
        return self.value > other.value


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
        self.step_x = (self.max_x - self.min_x) / (self.dim_x - 1)
        self.step_y = (self.max_y - self.min_y) / (self.dim_y - 1)

    def coord_to_index(self, coord):
        grid_coord = math.ceil(coord[0] / self.step_x - self.min_x / self.step_x), \
                     math.ceil(coord[1] / self.step_y - self.min_y / self.step_y)
        if self.min_x <= coord[0] <= self.max_x and self.min_y <= coord[1] <= self.max_y:
            return grid_coord
        return None

    def index_to_coord(self, indices):
        return self.grid.X[indices[0, :]], self.grid.Y[indices[:, 1]]

    def center(self):
        x, y = int((self.min_x + self.max_x)/2.0), int((self.min_y + self.max_y)/2.0)
        return x, y

    def center_index(self):
        return self.coord_to_index(self.center())


class GridBasedEnvironmentRepresentation(AbstractEnvironmentRepresentation):
    def __init__(self, grid):
        AbstractEnvironmentRepresentation.__init__(self)
        self.grid = grid

    def get_cost(self, coord):
        ix, iy = self.grid.coord_to_index(coord)
        return self.grid.values[ix, iy]

    def expand_strategy(self, state):
        # Expand all nodes in neighbors
        x, y = state.desc_coord
        neighbors = []
        indices_x = [x - self.grid.step_x, x, x + self.grid.step_x]
        indices_y = [y - self.grid.step_y, y, y + self.grid.step_y]
        for ix in indices_x:
            for iy in indices_y:
                n = self.grid.coord_to_index((ix, iy))
                if n is not None and not (state.coord[0] == ix and state.coord[1] == iy):
                    neighbors += [StateTreeNode(n, (ix, iy), 0.0, state)]
        return neighbors
