import numpy as np

from lqr import LQRPlanner, PositionalErrorSystemModel

import matplotlib.pyplot as plt

def main():
    system_model = PositionalErrorSystemModel(0.1)
    lqrplanner = LQRPlanner(system_model)
    starts = np.array([[0.0, 0.0],
                      [1.0, 0.0],
                      [0.0, 0.0]
                      ])
    goals = np.array([
        [20, 4],
        [25, 6],
        [18, -7]
    ])
    for i in range(len(goals)):
        start = starts[i].reshape(2,1)
        goal = goals[i].reshape(2,1)
        lqrplanner.plan(start, goal)
        tr = np.stack(lqrplanner.getTrajectory())
        plt.plot(tr[:,0], tr[:, 1])
        plt.plot(start[0], start[1], 'r^')
        plt.plot(goal[0], goal[1], 'r^')
    plt.show()

if __name__=="__main__":
    main()