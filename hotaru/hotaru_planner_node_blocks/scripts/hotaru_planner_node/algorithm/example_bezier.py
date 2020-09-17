'''
Created on Sep 16, 2020

@author: kyberszittya
'''

import matplotlib.pyplot as plt

import numpy as np

from bezier import BezierCurve


def main():
    bc = BezierCurve()
    points = np.array(
        [[0.0,   0.0],
         [6.0,   4.0],
         [7.0,  -2.0],
         [10.0, -5.0],
         [9.0,  -9.0]]
    )
    bc.add_control_vertices(points)
    bc.initialize_parameter_values()
    tr = bc.generate_path(100)
    plt.plot(tr[:,0], tr[:,1])
    plt.plot(points[:,0], points[:,1], 'r^')    
    plt.show()   


if __name__=="__main__":
    main()