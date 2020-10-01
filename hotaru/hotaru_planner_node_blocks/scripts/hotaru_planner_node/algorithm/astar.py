import numpy as np
from queue import PriorityQueue
from hotaru_planner_node.algorithm.heuristics import euclidean_distance

class SearchNode(object):

    def __init__(self, x, y, cost, parent):
        self.x = x
        self.y = y
        self.cost = cost
        self.parent = parent
        """   
        o - o - o
        |   |   |
        o - X - o
        |   |   |
        o - o - o
        """
        self.neighbor_indices = [
            ( 0,  1),
            ( 0, -1),
            ( 1,  1),
            (-1, -1),
            ( 1,  0),
            (-1,  0),
            (-1,  1),
            ( 1, -1)
        ]
        
    
    
    def get_neighbors(self):
        return self.neighbor_indices
            
    def get_index(self):
        return (self.x, self.y)
            
        

class AstarPlanner(object):

    def __init__(self):
        pass

    def set_configuration(self, start, goal):
        self.start = start
        self.goal = goal

    def set_grid(self, grid, resolution_x, resolution_y, offset_x, offset_y):
        self.grid = grid
        self.resolution_x = resolution_x
        self.resolution_y = resolution_y
        self.offset_x = offset_x
        self.offset_y = offset_y
        self.grid_shape = self.grid.shape
    
    
    def get_global_coord(self, ix, iy):
        return ix / self.resolution_x, iy / self.resolution_y 
    
    
    def get_grid_index(self, x, y):
        return 

    def plan(self):
        start_node = SearchNode(self.start[0], self.start[1], None)
        goal_node = SearchNode(self.goal[0], self.goal[1], None)
        fringe = PriorityQueue([start_node])
        visited = set()
        while len(fringe) > 0:
            current_node = fringe.pop()
            current_node_index = current_node.get_index()
            x = current_node_index[0]
            y = current_node_index[1]            
            visited.add(current_node.get_index())        
            for node in current_node.get_neighbors():
                print(node.get_index)
                i = node.get_index()
                nx, ny = i[0] + x , i[1] + y
                if (nx < self.grid_shape[0] and ny < self.grid_shape[1]
                    and nx > 0 and ny > 0):
                    cost = self.grid[nx, ny] + euclidean_distance(self)
                    new_node = SearchNode(current_node_index[0],
                        current_node_index[1], cost, current_node)
                    fringe.add(new_node)
            
            