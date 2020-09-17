import numpy as np

from hotaru_planner_node.algorithm.test_scenario_generator import test_case_generate_graphs

import matplotlib.pyplot as plt

def main():
    mesh_map_xx, mesh_map_yy, z, obs = test_case_generate_graphs(0.05, 0.2)
    plt.contourf(mesh_map_xx, mesh_map_yy, z)
    plt.show()

if __name__=="__main__":
    main()