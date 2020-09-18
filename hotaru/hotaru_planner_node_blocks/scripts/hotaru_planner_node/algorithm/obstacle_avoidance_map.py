'''
Created on Sep 17, 2020

@author: kyberszittya
'''

import numpy as np
from hotaru_planner_node.algorithm.heuristics import obstacle_avoidance_euclidean_distance,\
    calc_cv_weights

import matplotlib.pyplot as plt
from hotaru_planner_node.algorithm.bezier import RationalBezierCurve
from hotaru_planner_node.algorithm.test_scenario_generator import test_case_generate

import time

def main():
    mesh_map_xx, mesh_map_yy, z, obs = test_case_generate(0.05, 0.2)
    plt.contourf(mesh_map_xx,mesh_map_yy, z)
    # Bezier weights according to distance to objects
    ref_trajectory_points = np.array([
        [-5, 0],
        [0,  0],
        [3,  0],
        [5,  4],
        [8,  4],
        [12, 0],
        [15, 0]
    ])    
    start = time.time()
    cv_weights = calc_cv_weights(obs, ref_trajectory_points, obstacle_avoidance_euclidean_distance)

    bezier = RationalBezierCurve()
    bezier.add_control_vertices(ref_trajectory_points)
    bezier.initialize_parameter_values()
    bezier.set_weights(cv_weights)
    tr = bezier.generate_path(50)
    end = time.time()
    print(end - start)
    print(cv_weights)

    for i in range(len(tr) - 1):
        x = tr[i][0]
        y = tr[i][1]
        d = tr[i + 1] - tr[i]
        d /= np.linalg.norm(d)
        plt.plot((x, x+d[0]), (y, y+d[1]), color='yellow', alpha=0.3)
        #print(d[0]/d[1])
        
    plt.plot(tr[:,0], tr[:,1])
    # Control points
    plt.plot(ref_trajectory_points[:,0], ref_trajectory_points[:,1], 'r^')
    plt.show()
    
if __name__=="__main__":
    main()