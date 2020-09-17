'''
Created on Sep 17, 2020

@author: kyberszittya
'''

import numpy as np
from hotaru_planner_node.algorithm.heuristics import obstacle_avoidance_euclidean_distance,\
    calc_cv_weights

import matplotlib.pyplot as plt
from hotaru_planner_node.algorithm.bezier import RationalBezierCurve

def main():
    obs = np.array([6, 1])
    mesh_map_x = np.linspace(-15, 15, 150) 
    mesh_map_y = np.linspace(-15, 15, 150)
    mesh_map_xx, mesh_map_yy = np.meshgrid(mesh_map_x, mesh_map_y)
    z = obstacle_avoidance_euclidean_distance(obs[0], obs[1], mesh_map_xx, mesh_map_yy)
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
    cv_weights = calc_cv_weights(obs, ref_trajectory_points, obstacle_avoidance_euclidean_distance)
    print(cv_weights)
    bezier = RationalBezierCurve()
    bezier.add_control_vertices(ref_trajectory_points)
    bezier.initialize_parameter_values()
    bezier.set_weights(cv_weights)
    tr = bezier.generate_path(100)    
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