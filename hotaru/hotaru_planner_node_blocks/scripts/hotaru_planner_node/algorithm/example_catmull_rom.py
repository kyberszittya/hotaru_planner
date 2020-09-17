import numpy as np

from hotaru_planner_node.algorithm.catmull_rom import CatmullRomSpline
from lqr import LQRPlanner, PositionalErrorSystemModel

import matplotlib.pyplot as plt

def main():
    points = np.array(
        [[0.0, 0.0],
         [6.0, 4.0],
         [7.0, -2.0],
         [10.0, -5.0],
         [9.0, -9.0]]
    )
    tensions = [-1.5, -1.0, -0.5, -0.25, 0.0, 0.25, 0.5, 1.0, 1.5]
    axes = plt.gca()
    axes.set_xlim([0, 20])
    axes.set_ylim([10, -10])
    for tension in tensions:
        cv = CatmullRomSpline(tension=tension)
        cv.add_control_vertices(points)
        cv.initialize_parameter_values()
        tr = cv.generate_path(100)        
        line,  = plt.plot(tr[:, 0], tr[:, 1])
        line.set_label("T: {0}".format(tension))
        # Draw orientation
        for v in tr:
            x = v[0]+2*np.cos(v[2])
            y = v[1]+2*np.sin(v[2])            
            plt.plot((x,v[0]), (y, v[1]), color='purple', alpha=0.3)
        plt.plot(points[:, 0], points[:, 1], 'r^')
    plt.legend()
    plt.show()


if __name__=="__main__":
    main()
    