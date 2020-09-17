'''
Created on Sep 17, 2020

@author: kyberszittya
'''

from catmull_rom import CatmullRomSpline

import numpy as np

import matplotlib.pyplot as plt

def main():
    points = np.array([
        [0.0, 0.0],
        [4.0, 0.0],
        [18.0, 4.0],
        [24.0, 4.0],
        [35.0, 0.0],
        [40.0, 0.0]
        ])
    tensions = [-1.5, -1.0, -0.5, -0.25, 0.0, 0.25, 0.5, 1.0, 1.5]
    for tension in tensions:
        cv = CatmullRomSpline(tension=tension)
        cv.add_control_vertices(points)
        cv.initialize_parameter_values()
        tr = cv.generate_path(100)
        plt.plot(tr[:,0], tr[:, 1])
    plt.plot(points[:,0], points[:, 1], 'r^')
    plt.title("Obstacle avoidance path (Catmull-Rom)")
    plt.show()
    
if __name__=="__main__":
    main()
    