'''
Created on Sep 21, 2020

@author: kyberszittya
'''

import math

from virtual_lane_algorithm import ObstacleRepresentation, Obstacle, VehicleModel, VirtualLaneMotionPlanner

import matplotlib.pyplot as plt

import numpy as np

def generate_waypoints():
    waypoints = np.array([
        [0.0, 0.0],
        [5.0, 0.0],
        [7.0, 0.0],
        [10.0, 0.0],
        [15.0, 0.0],
        [20.0, 0.0],
        [30.0, 0.0]
    ])
    return waypoints

def test_object_detected():
    representation = ObstacleRepresentation()
    wheel_angle = 30.5*math.pi/180.0    
    vehicle_model = VehicleModel(wheel_angle, -wheel_angle, 2.5)
    vpl = VirtualLaneMotionPlanner(representation, vehicle_model)
    wps = generate_waypoints()
    vpl.set_reference_trajectory(wps)
    
    # Add obstacle
    c0 = Obstacle((10.0, 0), 0.7)
    representation.addObstacle(c0)
    s, d = vpl.obstacle_detected()
    assert(s)
    print("Detected obstacle at: {0}".format(d))
    
    # Plotting magic
    robot_grp = plt.Circle((0, 0), vehicle_model.robot_footprint_circle_radius, color='b')
    circle1 = plt.Circle((c0.position[0], c0.position[1]), c0.obstacle_radius, color='r')
    axes = plt.gca()
    axes.set_xlim([0, 20])
    axes.set_ylim([-10, 10])
    plt.plot(wps[:, 0], wps[:, 1])
    axes.add_artist(circle1)
    axes.add_artist(robot_grp)
    
    vpl.plan_cycle()
    plt.show()
    
    
def main():
    test_object_detected()
    
if __name__=="__main__":
    main()