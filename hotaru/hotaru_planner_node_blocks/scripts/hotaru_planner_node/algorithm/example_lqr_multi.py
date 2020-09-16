import numpy as np

from hotaru_planner_node.algorithm.bezier import BezierCurve, RationalBezierCurve
from hotaru_planner_node.algorithm.catmull_rom import CatmullRomSpline
from hotaru_planner_node.algorithm.hotaru_blocks import sparse_points, PlannerNode
from lqr import LQRPlanner, PositionalErrorSystemModel

import matplotlib.pyplot as plt

def main():
    system_model = PositionalErrorSystemModel(0.1)
    lqrplanner = LQRPlanner(system_model)
    start = np.array([[0.0, 0.0]]).reshape(2,1)
    goal = np.array([[20, 4]]).reshape(2,1)
    interpolation = CatmullRomSpline()
    planner = PlannerNode(lqrplanner, interpolation)
    planner.calc_plan(start, goal)
    tr = planner.generate_path()
    plt.plot(tr[:, 0], tr[:, 1])
    plt.plot(start[0], start[1], 'r^')
    plt.plot(goal[0], goal[1], 'r^')
    # Calc with Bezier
    interpolation2 = BezierCurve()
    planner2 = PlannerNode(lqrplanner, interpolation2)
    planner2.calc_plan(start, goal)
    tr = planner2.generate_path()
    plt.plot(tr[:, 0], tr[:, 1])
    # Calc with rational Bezier
    interpolation3 = RationalBezierCurve()
    planner3 = PlannerNode(lqrplanner, interpolation3)
    # TODO: solve curvature based weights
    planner3.calc_plan(start, goal)
    tr = planner2.generate_path()
    plt.plot(tr[:, 0], tr[:, 1])
    plt.show()

if __name__=="__main__":
    main()