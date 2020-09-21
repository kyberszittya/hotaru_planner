'''
Created on Sep 21, 2020

@author: kyberszittya
'''
from Queue import PriorityQueue

from hotaru_planner_node.algorithm.catmull_rom import CatmullRomSpline

import numpy as np


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


class DynamicLanePolygon(PolygonRepresentation):

    def __init__(self, lane_width, steps_width, horizon_steps):
        self.reference_trajectory = []
        self.lane_width = lane_width
        self.spline = CatmullRomSpline(0.5, 0.2)
        self.steps_width = steps_width
        self.horizon_steps = horizon_steps
        # Grid representation
        self.poly_vertices = np.zeros((self.horizon_steps, self.steps_width, 2))
        # Face status
        # 0: indicates no occupance
        # 1: indicate occupance
        #self.poly_faces = np.zeros((self.horizon_steps - 1, self.steps_width - 1, 3))
        self.obstacle_grid = np.zeros((self.horizon_steps, self.steps_width))


    def get_faces(self, indices):
        face_list = []
        for n in indices:
            face = []
            for t in n:
                face.append(self.poly_vertices[t[0], t[1], :])
            face_list.append(face)
        return np.stack(face_list)

    def get_obstacle_grid_indices(self, obstacle):
        # TODO: I know this sucks, but will use different method anyway
        obstacles_indices = []
        for i in range(self.horizon_steps - 1):
            for j in range(self.steps_width - 1):
                v0 = self.poly_vertices[i, j]
                v1 = self.poly_vertices[i + 1, j]
                v2 = self.poly_vertices[i + 1, j + 1]
                v3 = self.poly_vertices[i, j + 1]
                d0, d1, d2, d3 = self.inside_polygon(v0, v1, v2, v3, obstacle.position, obstacle.obstacle_radius)
                if d0 < 0.0 and d1 < 0.0 and d2 < 0.0 and d3 < 0.0:
                    indici = [(i, j), (i + 1, j), (i, j + 1), (i + 1, j + 1)]
                    obstacles_indices.append(indici)
                    for o in indici:
                        self.obstacle_grid[o[0], o[1]] = 1.0
        return obstacles_indices

    def set_reference_trajectory(self, reference_trajectory):
        self.reference_trajectory = reference_trajectory
        self.spline.add_control_vertices(reference_trajectory)
        self.spline.initialize_parameter_values()

    def calc(self):
        traj = self.spline.generate_path(self.horizon_steps)
        tangent = self.spline.generate_dpath(self.horizon_steps)
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

    def __init__(self, model, representation):
        self.vehicle_model = model
        self.environment_representation = representation
        # MAGIC NUMBER WARNING
        self.current_lane = 1
        self.neighbor_indices = [
            # X,Y, COST
            (1,  0, 1),
            (1,  1, 2),
            (1, -1, 2),
            (2,  1, 3),
            (2, -1, 3)
        ]

    def set_lane_position(self, lane_num):
        self.current_lane = lane_num

    def update_trajectory(self, trajectory):
        self.environment_representation.set_reference_trajectory(trajectory)

    def plan(self):
        fringe = PriorityQueue()
        fringe.put((0, AstarNode((0, self.current_lane))))
        visited = set()
        trajectory = []
        while not fringe.empty():
            new_node = fringe.get()
            if new_node[1].x == self.environment_representation.horizon_steps - 1:
                # Goal found!
                p = new_node[1]
                while not p.parent is None:
                    trajectory.append((p.x, p.y))
                    p = new_node[1].parent
                return trajectory
            for n in self.neighbor_indices:
                cost = new_node[0] + n[2]
                nx = new_node[1].x + n[0]
                ny = new_node[1].y + n[1]
                if ny >= self.environment_representation.steps_width or ny > 0 or \
                        nx >= self.environment_representation.horizon_steps:
                    continue

                fringe.put((cost, AstarNode((nx, ny), new_node[1])))
