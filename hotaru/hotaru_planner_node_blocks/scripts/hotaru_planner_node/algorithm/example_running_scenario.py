import numpy as np

from dynamic_lane_astar import DynamicLaneAstarPlanner, DynamicLanePolygon


def moving_scenario():
    planner = DynamicLaneAstarPlanner()
    lane_width = 3.0
    lane_polygon = DynamicLanePolygon(lane_width, 5, 15, 100)
    trajectory = np.array([
        [0.0, 0.0],
        [2.0, 1.0],
        [5.0, 0.0],
        [7.0, 0.0],
        [10.0, 0.5],
        [13.0, 0.0],
        [15.0, 0.0],
        [20.0, 0.0],
        [25.0, 0.0],
        [30.0, 0.0],
        [35.0, 0.0],
    ])
    for rv in poly_vertices:
        plt.scatter(rv[:, 0], rv[:, 1], color="purple", alpha=0.5)


def main():
    moving_scenario()

if __name__=="__main__":
    main()