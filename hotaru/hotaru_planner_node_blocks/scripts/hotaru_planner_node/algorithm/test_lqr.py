'''
Created on Sep 15, 2020

@author: kyberszittya
'''

from lqr import dare_solve

import numpy as np
from hotaru_planner_node.algorithm.lqr import LQRPlanner,\
    PositionalErrorSystemModel

import nose

import matplotlib.pyplot as plt

from numpy.testing import assert_almost_equal

import time


def test_dare():
    A = np.array([
        [-0.9, -0.3],
        [0.7, 0.1]
        ])
    B = np.array([1, 1]).reshape(2,1)
    Q = np.array([[1, 0],
                  [0, 3]])
    R = np.array([0.1])
    X = dare_solve(A, B, Q, R, 150, 0.001)
    assert_almost_equal(X[0,0],4.76848476, 0.001)
    assert_almost_equal(X[0,1],0.94378297, 0.001)
    assert_almost_equal(X[1,0],0.94378297, 0.001)
    assert_almost_equal(X[1,1],3.23691806, 0.001)
    print(X)
    
def test_lqr_basic():
    # Initialize system model
    system_model = PositionalErrorSystemModel(0.1)
    # Initialize planner
    lqrplanner = LQRPlanner(system_model)
    
    axes = plt.gca()
    axes.set_xlim([-20,20])
    axes.set_ylim([-20,20])
    mission_configurations = [((0.0,0.0), (20.0, 4.0)),
             ((-5.0,0.0), (15.0, 6.0)),
             ((-5.0,-6.0), (15.0, -3.0))]
    for g in mission_configurations:
        start = np.array([g[0][0], g[0][1]]).reshape(2,1)
        goal = np.array([g[1][0], g[1][1]]).reshape(2,1)
        start_time = time.time()
        lqrplanner.plan(start, goal)
        print("Elapsed time: {0}".format(time.time() - start_time))
        tr = np.stack(lqrplanner.getTrajectory())
        plt.plot(tr[:,0],tr[:,1])
        plt.plot(goal[0], goal[1], 'r^')
        plt.plot(start[0], start[1], 'r^')
    plt.show()
    

def main():
    test_dare()
    test_lqr_basic()
    
if __name__=="__main__":
    main()