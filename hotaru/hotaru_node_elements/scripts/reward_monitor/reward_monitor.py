'''
Created on May 14, 2020

@author: kyberszittya
'''

import rospy

from std_msgs.msg import Float64

from geometry_msgs.msg import TwistStamped, PoseStamped, Pose

from hotaru_msgs.msg import RefinedTrajectory

from enum import Enum
import math

class EpochState(Enum):
    RUNNING = 0
    START = 1
    END = 2

class DistanceState(Enum):
    FAR = 0
    CLOSE = 1
    COLLISION = 2
    
dict_events = {
    'time': 1.0,
    'collision': 2*10**9,
    'close_decay': 20
}

THRESHOLD_CLOSE = 5.0
DT = 0.1

def distancePoint(p0, p1):
    dx = p0.position.x-p1.position.x 
    dy = p0.position.y-p1.position.y
    return math.sqrt(dx*dx + dy*dy)

class RewardMonitor(object):
    
    def __init__(self):
        self.distance_state = DistanceState.FAR
        self.epoch_state = EpochState.START
        self.rewards = []
        self.current_reward = 0
        #
        self.sub_trajectory = rospy.Subscriber("/input_trajectory", RefinedTrajectory, self.cb)
        self.sub_velocity = rospy.Subscriber("/current_pose", PoseStamped, self.cbCurrentPose)
        self.sub_velocity = rospy.Subscriber("/current_velocity", TwistStamped, self.cbCurrentVelocity)
        self.sub_distance = rospy.Subscriber("/gazebo_sut_distance/distance", Float64, self.cbDistance)
        #
        self.goal_point = Pose()
        self.goal_point.position.x = 38.0
        self.goal_point.position.y = -38.0
        self.goal_point.position.z = 0.0
        self.sim_timer = rospy.Timer(rospy.Duration(DT), self.cbTimer)
        
    
    def start(self):
        self.sim_timer.start()
    
    def cbRefinedTrajectory(self, data):
        self.input_trajectory = data
    
    def cbCurrentVelocity(self, data):
        self.velocity = data.twist
        
    def cbCurrentPose(self, data):
        self.pose = data.pose
        
    def cbDistance(self, data):
        self.min_distance = data.data
        # Check for collision
        if self.min_distance < 0.05:
            self.distance_state = DistanceState.COLLISION
        # Check for close proximity
        elif self.min_distance < THRESHOLD_CLOSE:
            self.distance_state = DistanceState.CLOSE
        elif self.min_distance >= THRESHOLD_CLOSE:
            self.distance_state = DistanceState.FAR
    
    def updateReward(self):
        self.current_reward -= DT*dict_events["time"]
        if self.distance_state == DistanceState.CLOSE:
            self.current_reward -= DT*dict_events["close_decay"]
        elif self.distance_state == DistanceState.COLLISION:
            self.current_reward -= DT*dict_events["collision"]
        rospy.loginfo(self.current_reward)
    
    def reset_simulation(self):
        rospy.loginfo("Test case end, reseting simulation")
        
    def transitStart(self):
        self.reset_simulation()
        self.epoch_state = EpochState.START
    
    def transitEnd(self):
        self.rewards.append(self.current_reward)
        self.current_reward = 0.0
        self.epoch_state = EpochState.END   
    
        
    
    def cbTimer(self, event):
        if self.epoch_state == EpochState.START:
            if math.fabs(self.velocity.linear.x) >= 0.1 and distancePoint(self.goal_point, self.pose.pose) > 5.0:
                self.epoch_state = EpochState.RUNNING
        elif self.epoch_state == EpochState.RUNNING:
            if distancePoint(self.goal_point, self.pose.pose) < 1.0:
                self.transitEnd()
            self.updateReward()
        elif self.epoch_state == EpochState.END:
            self.transitStart()
            
            
            

def main():
    rew = RewardMonitor()
    rew.start()
    
if __name__=="__main__":
    main()
    