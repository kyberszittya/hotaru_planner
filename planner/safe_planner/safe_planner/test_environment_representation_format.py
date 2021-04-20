from safe_planner.environment_representation_format import Grid, GridBasedEnvironmentRepresentation, StateTreeNode

import numpy as np

import pytest


def generate_test_grid_10x10_1():
    t_X = np.arange(-10, 10 + 1, 1)
    t_Y = np.arange(-10, 10 + 1, 1)
    X, Y = np.meshgrid(t_X, t_Y)
    values = np.zeros(X.shape)
    # Grid
    gr = Grid(X, Y, values)
    return gr


def generate_test_grid_10x10_01():
    t_X = np.arange(-10, 10, 0.1)
    t_Y = np.arange(-10, 10, 0.1)
    X, Y = np.meshgrid(t_X, t_Y)
    values = np.zeros(X.shape)
    # Grid
    gr = Grid(X, Y, values)
    return gr


def test_grid_environment_representation_format_indexing_01():
    gr = generate_test_grid_10x10_01()
    res = gr.coord_to_index((1, 2))
    assert (res[0] == 111)
    assert (res[1] == 121)


def test_grid_environment_representation_format_indexing():
    gr = generate_test_grid_10x10_1()
    res = gr.coord_to_index((1, 2))
    assert (res[0] == 11)
    assert (res[1] == 12)


def test_grid_environment_representation_format():
    gr = generate_test_grid_10x10_1()
    assert (gr.center_index()[0] == 10)
    assert (gr.center_index()[1] == 10)


def test_grid_extreme_indexing():
    gr = generate_test_grid_10x10_1()
    res = gr.coord_to_index((10, 10))
    assert (res[0] == 20)
    assert (res[1] == 20)
    res = gr.coord_to_index((-10, -10))
    assert (res[0] == 0)
    assert (res[1] == 0)
    res = gr.coord_to_index((-10, 10))
    assert (res[0] == 0)
    assert (res[1] == 20)
    res = gr.coord_to_index((-11, 10))
    assert (res is None)


def test_grid_neighbors_simple():
    gr = generate_test_grid_10x10_1()
    gr_envrep = GridBasedEnvironmentRepresentation(gr)
    tree_node = StateTreeNode((0, 0), (0, 0), gr_envrep.get_cost((0, 0)))
    assert (tree_node.value == 0.0)
    center_neighbors = gr_envrep.expand_strategy(tree_node)
    assert (len(center_neighbors) == 8)
    tree_node = StateTreeNode((5, 5), (5, 5), gr_envrep.get_cost((5, 5)))
    assert (tree_node.value == 0.0)
    other_neighbors = gr_envrep.expand_strategy(tree_node)
    assert (len(other_neighbors) == 8)


def test_grid_neighbors_simple_indices():
    gr = generate_test_grid_10x10_1()
    gr_envrep = GridBasedEnvironmentRepresentation(gr)
    tree_node = StateTreeNode((0, 0), (0, 0), gr_envrep.get_cost((0, 0)))
    assert (tree_node.value == 0.0)
    center_neighbors = gr_envrep.expand_strategy(tree_node)
    assert (len(center_neighbors) == 8)
    # Assert indices are present
    expected_indices = [(9, 9), (9, 10), (10, 9), (9, 11),
                        (10, 11), (11, 9), (11, 10), (11, 11)]
    for n in center_neighbors:
        assert (n.coord in expected_indices)


def test_grid_neighbors_side():
    gr = generate_test_grid_10x10_1()
    gr_envrep = GridBasedEnvironmentRepresentation(gr)
    tree_node = StateTreeNode((10, 0), (10, 0), gr_envrep.get_cost((10, 0)))
    assert (tree_node.value == 0.0)
    center_neighbors = gr_envrep.expand_strategy(tree_node)
    assert (len(center_neighbors) == 5)


def test_grid_neighbors_corner():
    gr = generate_test_grid_10x10_1()
    gr_envrep = GridBasedEnvironmentRepresentation(gr)
    tree_node = StateTreeNode((10, 10), (10, 10), gr_envrep.get_cost((10, 10)))
    assert (tree_node.value == 0.0)
    center_neighbors = gr_envrep.expand_strategy(tree_node)
    print(center_neighbors)
    assert (len(center_neighbors) == 3)

