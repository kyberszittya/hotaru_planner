'''
Created on May 14, 2020

@author: kyberszittya
'''

import rospy

from std_msgs.msg import Float64

from geometry_msgs.msg import TwistStamped, PoseStamped, Pose

from hotaru_msgs.msg import RefinedTrajectory

from gazebo_msgs.msg import ModelStates
from std_srvs.srv import Empty

from enum import Enum
import math
from datetime import datetime
from collections import namedtuple

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
    'collision': 2*10**4,
    'close_decay': 20,
    'premature_end': 2000
}

THRESHOLD_CLOSE = 5.0
DT = 0.1

RewardRecord = namedtuple('RewardRecord', ['dt_ref', "dt_hysteresis", 'length', 'reward'])

def distancePoint(p0, p1):
    dx = p0.position.x-p1.position.x 
    dy = p0.position.y-p1.position.y
    return math.sqrt(dx*dx + dy*dy)

class RewardMonitor(object):
    
    def __init__(self, sut_name):
        self.t_params = {}
        self.sut_name = sut_name
        self.distance_state = DistanceState.FAR
        self.epoch_state = EpochState.START
        self.rewards = []
        self.current_reward = 0
        #
        self.sub_sim_state = rospy.Subscriber("/gazebo/model_states", ModelStates, self.cbModelState)
        self.sub_trajectory = rospy.Subscriber("/input_trajectory", RefinedTrajectory, self.cbRefinedTrajectory)
        self.sub_velocity = rospy.Subscriber("/current_pose", PoseStamped, self.cbCurrentPose)
        self.sub_velocity = rospy.Subscriber("/current_velocity", TwistStamped, self.cbCurrentVelocity)
        self.sub_distance = rospy.Subscriber("/gazebo_sut_distance/distance", Float64, self.cbDistance)
        self.pub_reward = rospy.Publisher("reward_function", Float64, queue_size=10)
        #
        self.reset = rospy.ServiceProxy("/gazebo/reset_world", Empty)        
        self.sim_timer = rospy.Timer(rospy.Duration(DT), self.cbTimer)
        self.start_epoch = rospy.Time.now()
        #
        self.msg_reward = Float64()
        
    def setGoal(self, goal):
        self.goal_point = Pose()
        self.goal_point.position.x = goal[0]
        self.goal_point.position.y = goal[1]
        self.goal_point.position.z = goal[2]
        
    
    def cbModelState(self, data):
        for i,m in enumerate(data.name):
            if m==self.sut_name:
                self.pose = data.pose[i]                
    
    def start(self, goal):
        #self.sim_timer.start()
        
        self.setGoal(goal)
        self.transitStart()
    
    def cbRefinedTrajectory(self, data):
        self.input_trajectory = data
    
    def cbCurrentVelocity(self, data):
        self.velocity = data.twist
    
    def cbCurrentPose(self, data):
        #self.pose = data.pose
        pass
        
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
        self.reset()
        self.start_epoch = rospy.Time.now()
        print("STARTING epoch")
        self.epoch_state = EpochState.START
        self.t_params["dt_ref"] = rospy.get_param("/hotaru_planner_node_teb_node/dt_ref")
        self.t_params["dt_hysteresis"] = rospy.get_param("/hotaru_planner_node_teb_node/dt_hysteresis")
        # Start time of epoch
        
    
    def transitEnd(self):
        print("Epoch ending")
        self.epoch_length = (rospy.Time.now() - self.start_epoch).to_sec()
        r = RewardRecord(dt_ref = self.t_params["dt_ref"], dt_hysteresis=self.t_params["dt_hysteresis"], length = self.epoch_length, reward = self.current_reward)
        self.rewards.append(r)
        print(r)
        self.current_reward = 0.0     
        self.epoch_state = EpochState.END   
    
        
    
    def cbTimer(self, event):
        if self.epoch_state == EpochState.START:
            if math.fabs(self.velocity.linear.x) >= 0.1 and distancePoint(self.goal_point, self.pose) > 5.0:
                self.epoch_state = EpochState.RUNNING
        elif self.epoch_state == EpochState.RUNNING:
            if distancePoint(self.goal_point, self.pose) < 1.0:
                self.transitEnd()
            self.updateReward()
            self.msg_reward.data = self.current_reward
            self.pub_reward.publish(self.msg_reward)
            # If the vehicle stopped, end the epoch with undesirable result
            if math.fabs(self.velocity.linear.x) < 0.1:
                self.current_reward -= self.rewards["premature_end"]
                self.transitEnd()
        elif self.epoch_state == EpochState.END:
            self.transitStart()
            
            
            

def main():
    rospy.init_node("reward_monitor")
    rew = RewardMonitor("nissanleaf")
    rew.start((-4, -32, 0))
    rospy.spin()
    with open("../data/rewards/{0}".format(datetime.now().strftime("%Y_%m_%d.%H_%M_%S.csv")), 'w') as f:
        f.write("dt_ref;dt_hysteresis;t_length;reward\n")
        for w in rew.rewards:
            print(w)
            f.write("{0},{1},{2},{3}\n".format(getattr(w,"dt_ref"),getattr(w,"dt_hysteresis"),getattr(w,"length"), getattr(w,"reward")))  
    
if __name__=="__main__":
    main()
    