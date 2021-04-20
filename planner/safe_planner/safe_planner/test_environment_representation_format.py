from safe_planner.environment_representation_format import Grid, GridBasedEnvironmentRepresentation

import numpy as np


def test_environment_representation_format():
    t_X = np.arange(-10, 10+1, 1)
    t_Y = np.arange(-10, 10+1, 1)
    X, Y = np.meshgrid(t_X, t_Y)
    values = np.zeros(X.shape)
    # Grid
    gr = Grid(X, Y, values)
    gr_env = GridBasedEnvironmentRepresentation(gr)
    print(X[10,11], Y[10, 11])
    print(gr.coord_to_index((1, 2)))
