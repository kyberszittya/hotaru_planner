'''
Created on Sep 21, 2020

@author: kyberszittya
'''
from Queue import PriorityQueue

from hotaru_planner_node.algorithm.catmull_rom import CatmullRomSpline

import numpy as np
from hotaru_planner_node.algorithm.heuristics import obstacle_avoidance_euclidean_distance

import time

from collections import deque

class Obstacle(object):

    def __init__(self, position, radius):
        self.position = position
        self.obstacle_radius = radius


class PolygonRepresentation(object):

    def line_distance(self, p, v0, v1):
        nz = np.linalg.norm(v0 - v1)
        nom = (v1[1] - v0[1]) * p[0] - (v1[0] - v0[0]) * p[1] + v1[0] * v0[1] - v1[1] * v0[0]
        return nom/nz

    def inside_polygon(self, v0, v1, v2, v3, p0, r=0.0):
        # Distances
        d0 = self.line_distance(p0, v0, v1) - r
        d1 = self.line_distance(p0, v1, v2) - r
        d2 = self.line_distance(p0, v2, v3) - r
        d3 = self.line_distance(p0, v3, v0) - r
        return d0, d1, d2, d3

class VehicleModel(object):

    def __init__(self, length, width):
        self.length = length
        self.width = width


class DynamicLanePolygon(PolygonRepresentation):

    def __init__(self, lane_width, steps_width, horizon_steps, lethal_val_scale=100.0,
                 non_lethal_val_scale=5.0):
        self.reference_trajectory = []
        self.lane_width = lane_width
        self.spline = CatmullRomSpline(0.0, 0.2)
        self.steps_width = steps_width
        self.horizon_steps = horizon_steps
        self.lethal_val_scale = lethal_val_scale
        self.non_lethal_val_scale = non_lethal_val_scale
        # TODO: make a better way to store obstacles
        self.obstacles_indices = []
        self.obstacles = []
        # Grid representation
        # Face status
        # 0: indicates no occupance
        # 1: indicate occupance
        #self.poly_faces = np.zeros((self.horizon_steps - 1, self.steps_width - 1, 3))
        self.obstacle_grid = None
        self.poly_vertices = None

    def reset_obstacle_list(self):
        self.obstacles = []

    def add_obstacle(self, obstacle):
        # TODO: I know this sucks, but will use different method anyway
        if self.poly_vertices is None:
            return None
        start = time.clock()
        for i in range(self.horizon_steps):
            for j in range(self.steps_width):
                d = obstacle_avoidance_euclidean_distance(
                    self.poly_vertices[i, j][0],
                    self.poly_vertices[i, j][1],
                    obstacle.position[0],
                    obstacle.position[1], obstacle_radius=0.0, val_scale=self.non_lethal_val_scale)
                self.obstacle_grid[i, j] += d
        for i in range(self.horizon_steps - 1):
            for j in range(self.steps_width - 1):
                # Check containment
                v = [self.poly_vertices[i, j],
                     self.poly_vertices[i + 1, j],
                     self.poly_vertices[i + 1, j + 1],
                     self.poly_vertices[i, j + 1]]
                d0, d1, d2, d3 = self.inside_polygon(v[0], v[1], v[2], v[3], obstacle.position,
                    obstacle.obstacle_radius)
                if d0 < 0.0 and d1 < 0.0 and d2 < 0.0 and d3 < 0.0:
                    indici = [(i, j), (i + 1, j), (i, j + 1), (i + 1, j + 1)]
                    self.obstacles_indices.append(indici)
                    for oi, o in enumerate(indici):
                        #print(self.val_scale*d)
                        self.obstacle_grid[o[0], o[1]] *= self.lethal_val_scale
        end = time.clock()
        self.obstacles.append(obstacle)

    def get_faces(self, indices):
        face_list = []
        if len(indices)==0:
            return face_list
        for n in indices:
            face = []
            for t in n:
                face.append(self.poly_vertices[t[0], t[1], :])
            face_list.append(face)
        return np.stack(face_list)

    def get_obstacle_grid_indices(self):
        return self.obstacles_indices

    def set_reference_trajectory(self, reference_trajectory):
        self.reference_trajectory = reference_trajectory
        self.spline.add_control_vertices(reference_trajectory)
        self.spline.initialize_parameter_values()
        
    def get_points(self, index_set):
        point_set = []
        for p in index_set:
            point_set.append(self.poly_vertices[p[0], p[1]])
        return np.stack(point_set)

    def calc(self):
        traj, tangent, self.ts, self.horizon_steps = self.spline.generate_uniform_path(2)
        self.obstacle_grid = np.zeros((self.horizon_steps, self.steps_width))
        self.obstacle_grid[:, self.steps_width - 1] = 1000
        self.obstacle_grid[:, 0] = 1000
        self.poly_vertices = np.zeros((self.horizon_steps, self.steps_width, 2))

        #traj = self.spline.generate_path(self.horizon_steps)
        #tangent = self.spline.generate_dpath(self.horizon_steps)
        normal = tangent[:,[1,0]]
        normal[:, 1] = -normal[:, 1]
        for i,no in enumerate(normal):
            self.poly_vertices[i, 0] = traj[i] + no * self.lane_width/2.0
            self.poly_vertices[i, 1] = traj[i]
            self.poly_vertices[i, 2] = traj[i] - no * self.lane_width/2.0
            self.poly_vertices[i, 3] = traj[i] - no * self.lane_width
            self.poly_vertices[i, 4] = traj[i] - no * 3 * self.lane_width/2.0
        return traj, tangent, normal, self.poly_vertices


class AstarNode(object):

    def __init__(self, i, parent=None):
        self.x, self.y = i
        self.parent = parent


class DynamicLaneAstarPlanner(object):

    def __init__(self, model, representation,
                 preferred_lane=1,
                 interp_weight_mult=3,
                 obstacle_scale = 100):
        self.vehicle_model = model
        self.environment_representation = representation
        self.current_lane = 1
        self.preferred_lane = preferred_lane
        self.interp_weight_mult = interp_weight_mult
        self.neighbor_indices = [
            # X,Y, COST
            (1,  0, 1),
            (1,  1, 2),
            (1, -1, 2),
            #(2,  1, 3),
            #(2, -1, 3)
        ]

    def find_closest_lane(self, pose):
        """
        Find the closest lane to a current position (Euclidean distance)

        :param pose: the pose to check
        :return: index of lane
        """
        distance = np.linalg.norm(pose - self.environment_representation.poly_vertices[0, :], axis=1)
        #print(distance)
        ind = np.argmin(distance)
        return ind

    def set_lane_position(self, lane_num):
        """
        Set lane number explicitly

        :param lane_num: new lane number
        :return:
        """
        self.current_lane = lane_num

    def update_trajectory(self, trajectory):
        self.environment_representation.set_reference_trajectory(trajectory)
        
    def lane_heuristics(self, x, current_lane):
        return np.abs(current_lane - self.preferred_lane)

    def plan(self):
        fringe = PriorityQueue()
        fringe.put((0, AstarNode((0, self.current_lane))))
        visited = set()        
        costs = {}
        trajectory = deque()
        node_cnt = 0
        while not fringe.empty():
            new_node = fringe.get()
            visited.add((new_node[1].x, new_node[1].y))
            if new_node[1].x == self.environment_representation.horizon_steps - 1:
                # Goal found!
                p = new_node[1]
                weights = deque()          
                while not p is None:
                    trajectory.appendleft((p.x, p.y))
                    weight = self.lane_heuristics(p.x, p.y)*self.interp_weight_mult+1
                    weight += self.environment_representation.obstacle_grid[p.x, p.y]
                    weights.appendleft(weight)
                    p = p.parent
                return trajectory, weights
            for n in self.neighbor_indices:                
                new_ind = (new_node[1].x + n[0], new_node[1].y + n[1])
                if new_ind[1] >= self.environment_representation.steps_width or new_ind[1] < 0 or \
                        new_ind[0] >= self.environment_representation.horizon_steps:
                    continue                
                if new_ind in visited:
                    continue
                h = self.lane_heuristics(new_ind[0], new_ind[1])
                cost = new_node[0] + n[2] + self.environment_representation.obstacle_grid[new_ind[0], new_ind[1]] + h
                if new_ind in costs:
                    if costs[new_ind] > cost:                                        
                        neigh = AstarNode(new_ind, new_node[1])
                        fringe.put((cost, neigh))
                else:
                    neigh = AstarNode(new_ind, new_node[1])
                    fringe.put((cost, neigh))
            node_cnt += 1


