'''
Created on Sep 21, 2020

@author: kyberszittya
'''

import numpy as np

import time

import matplotlib.pyplot as plt
from hotaru_planner_node.algorithm.dynamic_lane_astar import DynamicLanePolygon, Obstacle, DynamicLaneAstarPlanner
from hotaru_planner_node.algorithm.bezier import RationalBezierCurve


def example_with_traj(trajectory, c=Obstacle((10.1, 8.1), 0.7)):
    lane_width = 3.0
    c0 = c
    start = time.clock()
    lane_polygon = DynamicLanePolygon(lane_width, 5, 15, 100)
    lane_polygon.set_reference_trajectory(trajectory)
    env_repr_traj, tangent_traj, normal_traj, poly_vertices = lane_polygon.calc()
    grid_indices = lane_polygon.get_obstacle_grid_indices(c0)
    end = time.clock()
    obstacle_faces = lane_polygon.get_faces(grid_indices)
    print("Elapsed time to interpolate: {0}".format(end - start))
    # Plot everything
    axes = plt.gca()
    plt.plot(trajectory[:, 0], trajectory[:, 1])
    plt.plot(trajectory[:, 0], trajectory[:, 1], 'r^')
    plt.plot(env_repr_traj[:, 0], env_repr_traj[:, 1], 'purple')
    # Plot normal
    for rv in poly_vertices:
        plt.scatter(rv[:, 0], rv[:, 1], color="purple", alpha=0.5)
    for i, no in enumerate(normal_traj):
        plt.plot([env_repr_traj[i, 0], env_repr_traj[i, 0] + 1 * tangent_traj[i, 0]],
                 [env_repr_traj[i, 1], env_repr_traj[i, 1] + 1 * tangent_traj[i, 1]], 'blue')
        plt.plot([env_repr_traj[i, 0], env_repr_traj[i, 0] + 1 * no[0]],
                 [env_repr_traj[i, 1], env_repr_traj[i, 1] + 1 * no[1]], 'orange')

    for g in obstacle_faces:
        plt.plot(g[:, 0], g[:, 1], 'c^')
    print(grid_indices)
    axes.set_xlim([-2, 18])
    axes.set_ylim([-2, 18])
    circle1 = plt.Circle((c0.position[0], c0.position[1]), c0.obstacle_radius, color='r')
    axes.add_artist(circle1)
    # Plan!
    planner = DynamicLaneAstarPlanner(None, lane_polygon)
    start = time.clock()    
    traj, weights = planner.plan()
    print(weights)
    final_cv = lane_polygon.get_points(traj)
    end = time.clock()
    print("Elapsed time to plan: {0}".format(end - start))
    plt.plot(final_cv[:, 0], final_cv[:, 1], linewidth=2.0, linestyle='dashed')
    start = time.clock()
    final_waypoints = RationalBezierCurve()
    final_waypoints.add_control_vertices(final_cv)    
    final_waypoints.initialize_parameter_values()
    final_waypoints.set_weights(weights)
    f = final_waypoints.generate_path(20)    
    end = time.clock()
    print("Elapsed time to interpolate (curve): {0}".format(end - start))        
    plt.plot(f[:, 0], f[:, 1], linewidth=3.0)
    plt.show()


def ex_dynamic_lane_polygon_generation():
    # Basic setup
    trajectory = np.array([
        [0.0, 0.0],
        [0.5, 0.0],
        [1.0, 0.0],
        [2.0, 1.0],
        [5.0, 3.0],
        [7.0, 5.0],
        [10.0, 8.0],
        [7.0, 11.0],
        [5.0, 14.0]                
    ])
    example_with_traj(trajectory)


def ex_dynamic_lane_polygon_generation_straight():
    # Basic setup
    trajectory = np.array([
        [0.0, 0.0],
        [2.0, 1.0],
        [5.0, 0.0],
        [7.0, 0.0],
        [10.0, 0.5],
        [13.0, 0.0],
        [15.0, 0.0]
    ])
    example_with_traj(trajectory, Obstacle((10.1, 0.1), 0.7))


if __name__ == "__main__":
    ex_dynamic_lane_polygon_generation()
    ex_dynamic_lane_polygon_generation_straight()