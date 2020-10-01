import numpy as np

from hotaru_planner_node.algorithm.test_scenario_generator import test_case_generate_graphs

import matplotlib.pyplot as plt
from hotaru_planner_node.algorithm.astar import AstarPlanner

def main():
    mesh_map_xx, mesh_map_yy, z, obs = test_case_generate_graphs(0.05, 0.2)
    plt.contourf(mesh_map_xx, mesh_map_yy, z)
    planner = AstarPlanner()
    start_pos = np.array([0.0, 0.0])
    end_pos = np.array([10.0, 0.0])
    planner.set_configuration(start_pos, end_pos)
    planner.set_grid(z, 0.1, 0.1, np.min(mesh_map_xx), np.min(mesh_map_xx))    
    plt.show()

if __name__=="__main__":
    main()