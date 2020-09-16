'''
Created on Sep 16, 2020

@author: kyberszittya
'''

import matplotlib.pyplot as plt

import numpy as np

from bezier import BezierCurve
from hotaru_planner_node.algorithm.bezier import RationalBezierCurve

def main():
    bc = BezierCurve()
    points = np.array(
        [[0.0,   0.0],
         [2.0,   0.0],
         [6.0,   4.0],
         [7.0,   4.0],
         [9.0,  0.0],
         [15.0,  0.0]]
    )
    axes = plt.gca()
    axes.set_xlim([0, 15])
    axes.set_ylim([0, 15])
    bc.add_control_vertices(points)    
    tr = bc.generate_path(100)
    plt.plot(tr[:,0], tr[:,1])
    weights = [0.5, 0.8, 1.0, 1.2, 1.5, 1.75 ,2.5, 3.0, 4.0, 10.0]
    point_weights = [0.1, 1.2, 2.5, 2.5, 1.0, 2.0]    
    for wi in weights:
        point_weights[2] = wi
        point_weights[3] = wi
        bcr = RationalBezierCurve()
        bcr.add_control_vertices(points)
        bcr.set_weights(point_weights)
        tr = bcr.generate_path(100)
        plt.plot(tr[:,0], tr[:,1])
    plt.plot(points[:,0], points[:,1], 'r^')
    plt.show()   

if __name__=="__main__":
    main()


