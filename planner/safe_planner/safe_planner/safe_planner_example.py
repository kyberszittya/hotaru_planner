from safe_planner.safe_planner_impl import AStarPlannerImplementation, \
    GridBasedEnvironmentRepresentation, Grid

from fuzzy_network_engine.fuzzy_signature_quadtree import generate_elements, generate_quadtree, \
    FuzzySignatureElementFactory, FuzzySignatureEnvironmentRepresentation, \
    visualize_quadtree, visualize_infer_grid

import numpy as np


def test_simple_planning():
    rng = np.random.default_rng(0)
    # r_points = 100 * rng.random((10, 2)) - 50
    r_points = np.array([[-5.0, -5.0], [5.0, -5.0], [2.0, 3.0], [-6.0, 3.0], [-1.0, -3.0]])
    elements = generate_elements(FuzzySignatureElementFactory(), r_points)
    q_tree, intermediate_nodes, leaf_nodes = generate_quadtree(elements)
    # Infering on element
    env_repr = FuzzySignatureEnvironmentRepresentation(q_tree)
    obstacle_dimensions = [[5.0, 7.0],
                           [12.0, 12.0],
                           [10.0, 10.0],
                           [10.0, 10.0],
                           [5.0, 5.0]]
    for i, el in enumerate(elements[1:]):
        el.update_obstacle_boundary(obstacle_dimensions[i])
    env_repr.initialize_fuzzy_inference_system(elements)
    #visualize_quadtree(r_points, intermediate_nodes, leaf_nodes)
    X, Y, infer_grid, inference_result = env_repr.inference_grid(0.1)
    X_coarse, Y_coarse, coarse_grid, coarse_inference_result = env_repr.inference_grid(1.0)
    #visualize_infer_grid(X, Y, infer_grid, inference_result, X_coarse, Y_coarse, coarse_grid)
    grid = Grid(X_coarse, Y_coarse, infer_grid)
    grid_env_repr = GridBasedEnvironmentRepresentation(grid)



if __name__=="__main__":
    test_simple_planning()
