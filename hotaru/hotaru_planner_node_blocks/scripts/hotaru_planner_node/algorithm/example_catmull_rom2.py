import numpy as np

from hotaru_planner_node.algorithm.catmull_rom import CatmullRomSpline
from lqr import LQRPlanner, PositionalErrorSystemModel

import matplotlib.pyplot as plt


def main():
    points = np.array(
        [[0.0, 0.0],
         [4.0, 4.0],
         [7.0, 4.0],
         [10.0, 0.0],
         [12.0, 0.0]]
    )
    tensions = [-1.5, -1.0, -0.5, -0.25, 0.0, 0.25, 0.5, 1.0, 1.5]
    axes = plt.gca()
    axes.set_xlim([0, 12])
    axes.set_ylim([-6, 6])
    for tension in tensions:
        cv = CatmullRomSpline(tension=tension)
        cv.add_control_vertices(points)
        cv.initialize_parameter_values()
        tr = cv.generate_path(100)
        plt.plot(tr[:, 0], tr[:, 1])
        plt.plot(points[:, 0], points[:, 1], 'r^')
    plt.show()


if __name__=="__main__":
    main()