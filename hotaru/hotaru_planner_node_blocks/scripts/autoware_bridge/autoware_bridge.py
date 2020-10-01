'''
Created on Sep 16, 2020

@author: kyberszittya
'''

import rospy

from hotaru_planner_msgs.msg import Trajectory
from autoware_msgs.msg import Lane, LaneArray

class AutowareBridge(object):
    
    def __init__(self):
        pass
    
    def initialize_ros_interfaces(self):
        self.pub_autoware = rospy.Publisher('/lane', Lane, queue_size=1)
        self.sub_refined_trajectory = rospy.Subscriber('/refined_trajectory', 
            Trajectory, self.cb_refined_trajectory)
        
    def cb_refined_trajectory(self, data):
        self.trajectory = data
        self.pub_autoware.publish(self.trajectory)
        

def main():
    bridge = AutowareBridge()
    bridge.initialize_ros_interfaces()