'''
Created on Sep 21, 2020

@author: kyberszittya
'''

import numpy as np

import time

import matplotlib.pyplot as plt
from hotaru_planner_node.algorithm.dynamic_lane_astar import DynamicLanePolygon

def test_dyanmic_lane_polygon_generation():
    # Basic setup
    trajectory = np.array([
        [0.0, 0.0],
        [0.5, 0.0],
        [1.0, 0.0],
        [2.0, 1.0],
        
        [5.0, 3.0],
        [7.0, 5.0],
        [10.0, 8.0],
        [7.0, 11.0],
        [5.0, 14.0]                
    ])
    lane_width = 3.0
    start = time.clock()
    lane_polygon = DynamicLanePolygon(lane_width,6, 15)
    lane_polygon.set_reference_trajectory(trajectory)
    env_repr_traj, tangent_traj, normal_traj = lane_polygon.calc()
    end = time.clock()
    print("Elapsed time to interpolate: {0}".format(end-start))    
    # Plot everything
    axes = plt.gca()
    plt.plot(trajectory[:, 0], trajectory[:, 1])    
    plt.plot(trajectory[:, 0], trajectory[:, 1], 'r^')
    plt.plot(env_repr_traj[:, 0], env_repr_traj[:, 1], 'purple')
    # Plot normal
    for i,no in enumerate(normal_traj):
        plt.plot([env_repr_traj[i,0], env_repr_traj[i,0]+1*tangent_traj[i,0]],
                 [env_repr_traj[i,1], env_repr_traj[i,1]+1*tangent_traj[i,1]], 'blue')
        plt.plot([env_repr_traj[i,0], env_repr_traj[i,0]+1*no[0]],
                 [env_repr_traj[i,1], env_repr_traj[i,1]+1*no[1]], 'orange')
    axes.set_xlim([-2, 18])
    axes.set_ylim([-2, 18])
    plt.show()
    
if __name__=="__main__":
    test_dyanmic_lane_polygon_generation()