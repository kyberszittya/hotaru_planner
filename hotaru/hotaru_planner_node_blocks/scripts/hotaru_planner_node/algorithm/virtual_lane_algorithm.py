'''
Created on Sep 18, 2020

@author: kyberszittya
'''

import numpy as np

class VehicleModel(object):
    
    def __init__(self, max_angle, min_angle, robot_radius):
        self.max_angle = max_angle # Max wheel angle [rad]
        self.min_angle = min_angle # Min wheel angle [rad]
        self.robot_footprint_circle_radius = robot_radius # Radius [m]
        

class Obstacle(object):
    
    def __init__(self, position, radius):
        self.position = position
        self.obstacle_radius = radius


class ObstacleRepresentation(object):
    
    def __init__(self):
        self.obstacle_map = []
        
    def addObstacle(self, obstacle):
        self.obstacle_map.append(obstacle)
    
class VirtualLane(object):
    
    def __init__(self, trajectory_points):
        """
        
        Let's define a lane by two points
        
        """
        self.position = trajectory_points
    
    
    def set_waypoints(self, trajectory_points):
        self.position = trajectory_points
        
        
    def distance_to_lane(self, obstacle):
        """
        
        Distance of point from lane
        
        """
        # 
        x0 = obstacle.position[0]
        y0 = obstacle.position[1]
        #
        x1 = self.position[0][0]
        y1 = self.position[0][1]
        x2 = self.position[1][0]
        y2 = self.position[1][1]
        dx = x2 - x1
        dy = y2 - y1
        d = np.abs(dy*x0 - dx*y0 + x2*y1 - y2*x1)/np.sqrt(dx**2 + dy**2)
        return d 
            

class VirtualLaneMotionPlanner(object):
    
    def __init__(self, representation, motion_model):
        self.model = motion_model
        self.obstacle_representation = representation
        self.states = {"RELAY" : 0, "REPLAN": 1}
        self.control_state = self.states["RELAY"]
        self.virtual_lane_pos = 0
        self.ref_trajectory = None
        # Current lane should be the reference trajectory
        self.reference_lane = None
        self.current_lane = None        
        # FIFO queues to store left and right lanes
        self.left_lanes = []
        self.right_lanes = []
        
        
        
    def plan_cycle(self):
        if self.control_state==self.states["RELAY"]:
            if (self.obstacle_detected()[0]):
                self.evade()
        elif self.control_state==self.states["REPLAN"]:
            if (self.obstacle_detected()[0]):
                self.avoid()
    
    def set_reference_trajectory(self, waypoints):
        self.ref_trajectory = waypoints
        #
        points = waypoints[0:2]
        if (self.reference_lane == None):
            self.reference_lane = VirtualLane(points)
        else:
            self.reference_lane.set_waypoints(points)
        # If we are republishing original trajectory, set current lane
        if self.control_state==self.states["RELAY"]:
            self.current_lane = self.reference_lane
    
    
    def obstacle_detected(self):
        for o in self.obstacle_representation.obstacle_map:
            d = self.current_lane.distance_to_lane(o)  
            if d < self.model.robot_footprint_circle_radius :
                return True, d
        return False, 0
    
    def select_evasion_points(self):
        pass
        
        
    def evade(self):
        # Add current lane back to
        self.right_lanes.append(VirtualLane(0))        
        # TODO: handle Japanese/British lane rules as well
        self.left_lanes.append(VirtualLane())
        # If everything is OK, change state
        self.control_state = self.states["REPLAN"]
        
    def avoid(self):
        
        # State change
        if self.current_lane == self.reference_lane:
            self.control_state = self.states["RELAY"]

