'''
Created on Sep 21, 2020

@author: kyberszittya
'''
from hotaru_planner_node.algorithm.catmull_rom import CatmullRomSpline

import numpy as np

class DynamicLanePolygon(object):
    
    def __init__(self, lane_width, steps_width, horizon_steps):
        self.reference_trajectory = []
        self.lane_width = lane_width
        self.spline = CatmullRomSpline(0.0, 0.2)
        self.steps_width = steps_width
        self.horizon_steps = horizon_steps
            
    
    def set_reference_trajectory(self, reference_trajectory):
        self.reference_trajectory = reference_trajectory
        self.spline.add_control_vertices(reference_trajectory)
        self.spline.initialize_parameter_values()
        
    def calc(self):
        traj = self.spline.generate_path(self.horizon_steps)
        tangent = self.spline.generate_dpath(self.horizon_steps)
        normal = tangent[:,[1,0]]
        normal[:, 1] = -normal[:, 1]
        return traj, tangent, normal
        
        