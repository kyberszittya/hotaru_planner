import rclpy
from rclpy.node import Node

from hotaru_msgs.msg import Trajectory
from safe_planner.environment_representation_format import AbstractEnvironmentRepresentation


class AbstractPlannerImplementation(object):

    def __init__(self, env_repr):
        self.refined_trajectory = Trajectory()
        self.environment_snapshot = AbstractEnvironmentRepresentation()

    def update_trajectory(self, traj):
        self.refined_trajectory = traj


    def refine(self):
        raise NotImplementedError

    def return_trajectory(self):
        return self.refined_trajectory


class AStarPlannerImplementation(AbstractPlannerImplementation):

    def __init__(self, env_repr):
        AbstractPlannerImplementation.__init__(self, env_repr)

    def refine(self):
        visited = set()



class PlannerNode(Node):

    def __init__(self, name, planning_rate, planner):
        rclpy.Node.__init__(self, "planner_"+name)
        # Abstract implementation of planner (adapter)
        self.planner_adapter = planner
        # Basic I/O interface (input and refined trajectory)
        # Subscriber interface for trajectory planning
        self._refined_trajectory = self.create_publisher(Trajectory, 'refined_trajectory')
        self._input_trajectory = self.create_subscription(Trajectory, 'input_trajectory', self.cb_trajectory)
        # Planner timer cycle
        self._plan_timer = self.create_timer(planning_rate, self.cb_timer_planner)


    def cb_trajectory(self, msg):
        self.planner_adapter.update_trajectory(msg)

    def cb_timer_planner(self):
        self.planner_adapter.refine()



