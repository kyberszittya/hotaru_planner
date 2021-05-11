import rclpy
from geometry_msgs.msg import PoseStamped

from safe_planner.ros2_planner_node_elements import PlannerNode
from safe_planner.impl_graph_planner import AStarPlannerComponent
from safe_planner.util_tester_components import GoalTesterNode
from safe_planner.environment_representation_format import GridBasedEnvironmentRepresentation, Grid

from fuzzy_network_engine.fuzzy_signature_quadtree import generate_elements, generate_quadtree, \
    FuzzySignatureElementFactory, FuzzySignatureEnvironmentRepresentation, \
    visualize_quadtree, visualize_infer_grid

import numpy as np


def create_env_representation(boundary=np.array(
        [[-20, 20], [20, 20], [20, -20], [-20, -20]]), resolution=0.5):
    r_points = np.array([[-5.0, -5.0], [5.0, -5.0], [2.0, 3.0], [-6.0, 3.0], [-1.0, -3.0]])
    elements = generate_elements(FuzzySignatureElementFactory(), r_points)
    q_tree, intermediate_nodes, leaf_nodes = generate_quadtree(elements, boundary=boundary)
    env_repr = FuzzySignatureEnvironmentRepresentation(q_tree)
    obstacle_dimensions = [[5.0, 7.0],
                           [12.0, 12.0],
                           [10.0, 10.0],
                           [10.0, 10.0],
                           [5.0, 5.0]]
    for i, el in enumerate(elements[1:]):
        el.update_obstacle_boundary(obstacle_dimensions[i])
    env_repr.initialize_fuzzy_inference_system(elements)
    #X, Y, infer_grid, inference_result = env_repr.inference_grid(0.1)
    X_coarse, Y_coarse, coarse_grid, coarse_inference_result = env_repr.inference_grid(resolution)
    c_grid = 50.0
    grid = Grid(X_coarse, Y_coarse, coarse_grid * c_grid)
    env_grid = GridBasedEnvironmentRepresentation(grid)
    return env_grid


def create_coarse_planner():
    """
    Create a planner on higher-level to generate coarse trajectory

    :return:
    """
    # Step 1 define obstacles
    grid = create_env_representation()
    # Step 2 define planner
    astar_planner = AStarPlannerComponent(grid, 0.1)
    coarse_planner = PlannerNode("coarse_planner", 0.1, astar_planner)
    # For this test case set goal and start explicitly
    # Start setup
    start = PoseStamped()
    start.pose.position.x = -19.0
    start.pose.position.y = -19.0
    astar_planner.set_start_point(start)
    # Goal setup
    goal = PoseStamped()
    goal.pose.position.x = 19.0
    goal.pose.position.y = 19.0
    astar_planner.set_goal(goal)
    #
    return coarse_planner


def create_local_planner():
    """
    Create a planner on a local representation (same size as before)

    :return:
    """
    # Step 1 define environment representation
    grid = create_env_representation(np.array(
        [[-20, 20], [20, 20], [20, -20], [-20, -20]]), resolution=0.1)
    # Step 2 define planner
    astar_planner = AStarPlannerComponent(grid, 0.05, bypass_planning=False)
    local_planner = PlannerNode("local_planner", 0.1, astar_planner)
    # For this test case set goal and start explicitly
    # Start setup
    start = PoseStamped()
    start.pose.position.x = -19.0
    start.pose.position.y = -19.0
    astar_planner.set_start_point(start)
    # Goal setup
    goal = PoseStamped()
    goal.pose.position.x = 19.0
    goal.pose.position.y = 19.0
    astar_planner.set_goal(goal)
    #
    return local_planner


def main():
    """
    System
                large_grid                      small_grid
                  ^                                 ^
                  |                                 |
    [Goal] -> coarse_planner -> [Trajectory] -> local_planner -> [Final trajectory]

    """
    rclpy.init()
    coarse_planner = create_coarse_planner()
    local_planner = create_local_planner()
    local_planner.connect_to_node(coarse_planner)
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(coarse_planner)
    executor.add_node(local_planner)
    executor.spin()
    coarse_planner.destroy_node()
    local_planner.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
