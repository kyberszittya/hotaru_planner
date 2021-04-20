from safe_planner.composite_planner_impl import AStarPlannerImplementation
from safe_planner.environment_representation_format import GridBasedEnvironmentRepresentation, Grid

from fuzzy_network_engine.fuzzy_signature_quadtree import generate_elements, generate_quadtree, \
    FuzzySignatureElementFactory, FuzzySignatureEnvironmentRepresentation, \
    visualize_quadtree, visualize_infer_grid

from geometry_msgs.msg import PoseStamped

import matplotlib.pyplot as plt

import numpy as np


def test_simple_planning():
    rng = np.random.default_rng(0)
    # r_points = 100 * rng.random((10, 2)) - 50
    r_points = np.array([[-5.0, -5.0], [5.0, -5.0], [2.0, 3.0], [-6.0, 3.0], [-1.0, -3.0]])
    elements = generate_elements(FuzzySignatureElementFactory(), r_points)
    q_tree, intermediate_nodes, leaf_nodes = generate_quadtree(elements)
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
    X_coarse, Y_coarse, coarse_grid, coarse_inference_result = env_repr.inference_grid(0.5)
    #visualize_infer_grid(X, Y, infer_grid, inference_result, X_coarse, Y_coarse, coarse_grid)
    c_grid = 50.0
    grid = Grid(X_coarse, Y_coarse, coarse_grid * c_grid)
    grid_env_repr = GridBasedEnvironmentRepresentation(grid)
    print(grid.max_x)
    a_star_planner = AStarPlannerImplementation(grid_env_repr, 0.1)
    start = PoseStamped()
    start.pose.position.x = -10.0
    start.pose.position.y = -10.0
    a_star_planner.set_state(start)

    goal = PoseStamped()
    goal.pose.position.x = 11.5
    goal.pose.position.y = 11.5
    a_star_planner.set_goal(goal)
    a_star_planner.refine()
    res = a_star_planner.get_trajectory_array()
    # Visualize
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot_wireframe(X_coarse, Y_coarse, coarse_grid, rstride=1, cstride=1, color="green")
    ax.scatter(res[:, 0], res[:, 1], color='b')

    plt.show()


if __name__ == "__main__":
    test_simple_planning()
