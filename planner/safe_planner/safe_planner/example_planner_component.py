from safe_planner.impl_graph_planner import AStarPlannerComponent
from safe_planner.environment_representation_format import GridBasedEnvironmentRepresentation, Grid

from fuzzy_network_engine.fuzzy_signature_quadtree import generate_elements, generate_quadtree, \
    FuzzySignatureElementFactory, FuzzySignatureEnvironmentRepresentation, \
    visualize_quadtree, visualize_infer_grid

from geometry_msgs.msg import PoseStamped

import matplotlib.pyplot as plt

import numpy as np
import time


def test_simple_planning_astar():
    rng = np.random.default_rng(0)
    # r_points = 100 * rng.random((10, 2)) - 50
    r_points = np.array([[-5.0, -5.0], [5.0, -5.0], [2.0, 3.0], [-6.0, 3.0], [-1.0, -3.0]])
    elements = generate_elements(FuzzySignatureElementFactory(), r_points)
    q_tree, intermediate_nodes, leaf_nodes = generate_quadtree(elements,
                                                               boundary=np.array([[-20, 20], [20, 20], [20, -20], [-20, -20]]))
    # Inferring on element
    env_repr = FuzzySignatureEnvironmentRepresentation(q_tree)
    obstacle_dimensions = [[5.0, 7.0],
                           [12.0, 12.0],
                           [10.0, 10.0],
                           [10.0, 10.0],
                           [5.0, 5.0]]
    for i, el in enumerate(elements[1:]):
        el.update_obstacle_boundary(obstacle_dimensions[i])
    env_repr.initialize_fuzzy_inference_system(elements)
    X, Y, infer_grid, inference_result = env_repr.inference_grid(0.1)
    t0_env_infer = time.perf_counter()
    X_coarse, Y_coarse, coarse_grid, coarse_inference_result = env_repr.inference_grid(0.5)
    t1_env_infer = time.perf_counter()
    #visualize_infer_grid(X, Y, infer_grid, inference_result, X_coarse, Y_coarse, coarse_grid)
    print('Time to infer on environment representation (res 0.5): {0}'.format(t1_env_infer - t0_env_infer))
    c_grid = 50.0
    grid = Grid(X_coarse, Y_coarse, coarse_grid * c_grid)
    grid_env_repr = GridBasedEnvironmentRepresentation(grid)
    print(grid.max_x)
    a_star_planner = AStarPlannerComponent(grid_env_repr, 0.1)
    start = PoseStamped()
    start.pose.position.x = -19.0
    start.pose.position.y = -19.0
    a_star_planner.set_state(start)

    goal = PoseStamped()
    goal.pose.position.x = 19.0
    goal.pose.position.y = 19.0
    a_star_planner.set_goal(goal)
    t0_astar_plan = time.perf_counter()
    a_star_planner.refine()
    t1_astar_plan = time.perf_counter()
    print('Time to plan (A*): {0}'.format(t1_astar_plan - t0_astar_plan))
    res = a_star_planner.get_trajectory_array()
    # Visualize
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot_wireframe(X_coarse, Y_coarse, coarse_grid, rstride=1, cstride=1, color="green")
    ax.scatter(res[:, 0], res[:, 1], color='b')

    plt.show()


def test_fine_small_planning_astar():
    rng = np.random.default_rng(0)
    # r_points = 100 * rng.random((10, 2)) - 50
    r_points = np.array([[-5.0, -5.0], [5.0, -5.0], [2.0, 3.0], [-6.0, 3.0], [-1.0, -3.0]])
    elements = generate_elements(FuzzySignatureElementFactory(), r_points)
    q_tree, intermediate_nodes, leaf_nodes = generate_quadtree(elements,
                                                               boundary=np.array([[-20, 20], [20, 20], [20, -20], [-20, -20]]))
    # Inferring on element
    env_repr = FuzzySignatureEnvironmentRepresentation(q_tree)
    obstacle_dimensions = [[5.0, 7.0],
                           [12.0, 12.0],
                           [10.0, 10.0],
                           [10.0, 10.0],
                           [5.0, 5.0]]
    for i, el in enumerate(elements[1:]):
        el.update_obstacle_boundary(obstacle_dimensions[i])
    env_repr.initialize_fuzzy_inference_system(elements)
    t0_env_infer = time.perf_counter()
    X_coarse, Y_coarse, coarse_grid, coarse_inference_result = env_repr.inference_grid(0.1)
    t1_env_infer = time.perf_counter()
    #visualize_infer_grid(X, Y, infer_grid, inference_result, X_coarse, Y_coarse, coarse_grid)
    print('Time to infer on environment representation (res 0.1): {0}'.format(t1_env_infer - t0_env_infer))
    c_grid = 50.0
    grid = Grid(X_coarse, Y_coarse, coarse_grid * c_grid)
    grid_env_repr = GridBasedEnvironmentRepresentation(grid)
    print(grid.max_x)
    a_star_planner = AStarPlannerComponent(grid_env_repr, 0.1)
    start = PoseStamped()
    start.pose.position.x = -19.0
    start.pose.position.y = -19.0
    a_star_planner.set_state(start)

    goal = PoseStamped()
    goal.pose.position.x = 19.0
    goal.pose.position.y = 19.0
    a_star_planner.set_goal(goal)
    t0_astar_plan = time.perf_counter()
    a_star_planner.refine()
    t1_astar_plan = time.perf_counter()
    print('Time to plan (A*): {0}'.format(t1_astar_plan - t0_astar_plan))
    res = a_star_planner.get_trajectory_array()
    # Visualize
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot_wireframe(X_coarse, Y_coarse, coarse_grid, rstride=1, cstride=1, color="green")
    ax.scatter(res[:, 0], res[:, 1], color='b')

    plt.show()


if __name__ == "__main__":
    test_simple_planning_astar()
    #test_fine_small_planning_astar()
