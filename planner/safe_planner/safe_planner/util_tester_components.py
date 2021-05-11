from rclpy.node import Node

from hotaru_msgs.msg import Trajectory, Waypoint


class GoalTesterNode(Node):
    def __init__(self):
        Node.__init__("goal_tester_node")
        self._pub_goal = self.create_publisher(Trajectory, "initial_goal")