import numpy as np

from hotaru_planner_node.algorithm.heuristics import obstacle_avoidance_euclidean_distance


def test_case_generate(resolution_x, resolution_y):
    obs = np.array([6, 1])
    mesh_map_x = np.linspace(-15, 15, 150)
    mesh_map_y = np.linspace(-15, 15, 150)
    mesh_map_xx, mesh_map_yy = np.meshgrid(mesh_map_x, mesh_map_y)
    z = obstacle_avoidance_euclidean_distance(obs[0], obs[1], mesh_map_xx, mesh_map_yy)
    return mesh_map_xx, mesh_map_yy, z, obs


def test_case_generate_graphs(resolution_x, resolution_y):
    obs = np.array([
        [6, 1],
        [10, 6],
        [5, 4],
        [4.5, 15]
    ])
    mesh_map_x = np.linspace(-2, 15, 150)
    mesh_map_y = np.linspace(-2, 15, 150)
    mesh_map_xx, mesh_map_yy = np.meshgrid(mesh_map_x, mesh_map_y)
    z = None
    for o in obs:
        if z is None:
            z = obstacle_avoidance_euclidean_distance(o[0], o[1], mesh_map_xx, mesh_map_yy)
        else:
            z += obstacle_avoidance_euclidean_distance(o[0], o[1], mesh_map_xx, mesh_map_yy)
    return mesh_map_xx, mesh_map_yy, z, obs