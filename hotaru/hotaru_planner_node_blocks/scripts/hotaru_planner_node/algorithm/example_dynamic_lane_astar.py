'''
Created on Sep 21, 2020

@author: kyberszittya
'''

import numpy as np

import time

import matplotlib.pyplot as plt
from hotaru_planner_node.algorithm.dynamic_lane_astar import DynamicLanePolygon, Obstacle, \
    DynamicLaneAstarPlanner, VehicleModel
from hotaru_planner_node.algorithm.bezier import RationalBezierCurve


def example_bare_setup(trajectory, c0=[], start_lane=1):
    lane_width = 3.0
    lane_polygon = DynamicLanePolygon(lane_width, 5, 15, 100)
    lane_polygon.set_reference_trajectory(trajectory)
    env_repr_traj, tangent_traj, normal_traj, poly_vertices = lane_polygon.calc()
    for c in c0:
        lane_polygon.add_obstacle(c)
    grid_indices = lane_polygon.get_obstacle_grid_indices()
    obstacle_faces = lane_polygon.get_faces(grid_indices)
    planner = DynamicLaneAstarPlanner(None, lane_polygon)
    planner.set_lane_position(start_lane)
    for rv in poly_vertices:
        plt.scatter(rv[:, 0], rv[:, 1], color="purple", alpha=0.5)
    pose = np.array([0.05, 0.8])
    closest_lane = planner.find_closest_lane(pose)
    # Plot closest lane point
    plt.plot(poly_vertices[0, closest_lane, 0], poly_vertices[0, closest_lane, 1], 'b^')
    # Plot pose
    plt.plot(pose[0], pose[1], 'r^')

    axes = plt.gca()
    axes.set_xlim([-2, 18])
    axes.set_ylim([-2, 18])
    plt.show()



def example_with_traj(trajectory, c0=[], model=None, start_lane=1):
    lane_width = 3.0
    start = time.clock()
    lane_polygon = DynamicLanePolygon(lane_width, 5, 15, 100)
    lane_polygon.set_reference_trajectory(trajectory)
    env_repr_traj, tangent_traj, normal_traj, poly_vertices = lane_polygon.calc()
    for c in c0:
        lane_polygon.add_obstacle(c)
    grid_indices = lane_polygon.get_obstacle_grid_indices()
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
    for c in c0:
        circle1 = plt.Circle((c.position[0], c.position[1]), c.obstacle_radius, color='r')
        axes.add_artist(circle1)
    # Plan!
    planner = DynamicLaneAstarPlanner(None, lane_polygon)
    planner.set_lane_position(start_lane)

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
    example_with_traj(trajectory, [Obstacle((10.1, 8.1), 0.7)])


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
    example_with_traj(trajectory, [Obstacle((10.1, 0.1), 0.7)])


def ex_dynamic_lane_polygon_generation_straight_obstacle_end():
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
    example_with_traj(trajectory, [Obstacle((14.7, 0.1), 0.7)])


def ex_dynamic_lane_polygon_generation_no_obstacle():
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
    example_with_traj(trajectory)


def ex_dynamic_lane_polygon_generation_muiltiple_obstacle():
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
    example_with_traj(trajectory, [Obstacle((14.7, 0.1), 0.7), Obstacle((12.7, 0.1), 0.7)])


def ex_dynamic_lane_polygon_generation_muiltiple_obstacle2():
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
    example_with_traj(trajectory, [Obstacle((14.7, 0.1), 0.7),
                                   Obstacle((12.7, 0.1), 0.7),
                                   Obstacle((8.7, 4.1), 0.7)])

def ex_dynamic_lane_polygon_generation_muiltiple_obstacle3():
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
    example_with_traj(trajectory, [Obstacle((14.7, 0.1), 0.7),
                                   Obstacle((8.7, 0.1), 0.7)])


def ex_dynamic_lane_polygon_generation_different_starting_lane():
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
    example_with_traj(trajectory, [Obstacle((1.7, 0.1), 0.7)], start_lane=3)


def ex_dynamic_lane_polygon_generation_vehicle_model():
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
    model = VehicleModel(length=2.7, width=1.5)
    example_with_traj(trajectory, [Obstacle((14.7, 0.1), 0.7),
                                   Obstacle((8.7, 0.1), 0.7)], model=model)



def ex_planner_closest_waypoint():
    trajectory = np.array([
        [0.0, 0.0],
        [2.0, 1.0],
        [5.0, 0.0],
        [7.0, 0.0],
        [10.0, 0.5],
        [13.0, 0.0],
        [15.0, 0.0]
    ])
    example_bare_setup(trajectory, c0=[], start_lane=1)


if __name__ == "__main__":
    #ex_dynamic_lane_polygon_generation_no_obstacle()
    #ex_dynamic_lane_polygon_generation()
    #ex_dynamic_lane_polygon_generation_straight()
    #ex_dynamic_lane_polygon_generation_straight_obstacle_end()
    #ex_dynamic_lane_polygon_generation_muiltiple_obstacle()
    #ex_dynamic_lane_polygon_generation_muiltiple_obstacle2()
    #ex_dynamic_lane_polygon_generation_muiltiple_obstacle3()
    #ex_dynamic_lane_polygon_generation_different_starting_lane()
    #ex_planner_closest_waypoint()
    ex_dynamic_lane_polygon_generation_vehicle_model()