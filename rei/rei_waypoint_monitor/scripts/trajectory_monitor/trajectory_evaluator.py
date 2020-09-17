#! /usr/bin/env python
'''
Created on May 12, 2020

@author: kyberszittya
'''

import rospy

from hotaru_msgs.msg import RefinedTrajectory


class TrajectoryEvaluation(object):
    
    def __init__(self):
        self.sub_input_trajectory = rospy.Subscriber("/input_trajectory", RefinedTrajectory, self.cbInputTrajectory)
        self.sub_refined_trajectory = rospy.Subscriber("refined_trajectory", RefinedTrajectory, self.cbRefinedTrajectory)
    
    
    def cbInputTrajectory(self, data):
        self.input_trajectory = data
        
    def cbRefinedTrajectory(self, data):
        self.refined_trajectory = data
         


def main():
    eval_comp = TrajectoryEvaluation()