import numpy as np

from hotaru_planner_node.algorithm.hotaru_blocks import sparse_points
from lqr import LQRPlanner, PositionalErrorSystemModel

import matplotlib.pyplot as plt

def main():
    system_model = PositionalErrorSystemModel(0.1)
    lqrplanner = LQRPlanner(system_model)
    start = np.array([[0.0, 0.0]]).reshape(2,1)
    goal = np.array([[20, 4]]).reshape(2,1)
    lqrplanner.plan(start, goal)
    tr = np.stack(lqrplanner.getTrajectory())
    sparse_tr = sparse_points(tr)

    plt.plot(tr[:,0], tr[:, 1])
    plt.plot(tr[:, 0], tr[:, 1], "go")
    plt.plot(sparse_tr[:, 0], sparse_tr[:, 1], "bo")
    print(len(sparse_tr), len(tr))
    plt.plot(start[0], start[1], 'r^')
    plt.plot(goal[0], goal[1], 'r^')
    plt.show()

if __name__=="__main__":
    main()