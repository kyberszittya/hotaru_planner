'''
Created on Sep 14, 2020

@author: kyberszittya
'''

import rospy

from geometry_msgs.msg import PoseStamped, TwistStamped

import math

class ScenarioParkingPoints(object):
    
    """
    @param start: Start pose (x,y,yaw)
    @param goal: Goal pose (x,y,yaw)
    """
    def __init__(self, start=(0,0,0), goal=(0,0,0)):
        # Start pose
        self.start_point = PoseStamped()
        self.start_point.pose.position.x = start[0]
        self.start_point.pose.position.y = start[1]
        self.start_point.pose.position.z = 0.0
        self.start_point.pose.orientation.x = 0.0
        self.start_point.pose.orientation.y = 0.0
        self.start_point.pose.orientation.z = math.sin(start[2]/2)
        self.start_point.pose.orientation.w = math.cos(start[2]/2)
        self.start_point.header.frame_id = "map"
        # Goal pose 
        self.goal_point = PoseStamped()
        self.goal_point.pose.position.x = goal[0]
        self.goal_point.pose.position.y = goal[1]
        self.goal_point.pose.orientation.x = 0.0
        self.goal_point.pose.orientation.y = 0.0
        self.goal_point.pose.orientation.z = math.sin(goal[2]/2)
        self.goal_point.pose.orientation.w = math.cos(goal[2]/2)
        self.goal_point.pose.position.z = 0.0
        self.goal_point.header.frame_id = "map"
        # Pose
        self.pub_start_pose = rospy.Publisher("/current_pose", PoseStamped, queue_size=1)
        self.pub_goal_pose = rospy.Publisher("/goal_state", PoseStamped, queue_size=1) 
        # Timer
        
    def startTimer(self):
        self.timer_scenario = rospy.Timer(rospy.Duration(0.05), self.scenarioPublisher)
    
    
    def scenarioPublisher(self, event):
        self.start_point.header.stamp = rospy.Time.now()
        self.pub_start_pose.publish(self.start_point)
        self.goal_point.header.stamp = rospy.Time.now()
        self.pub_goal_pose.publish(self.goal_point)
        

def main():
    rospy.init_node("scenario_publisher")
    scen = ScenarioParkingPoints((0,0,0), (20,5,0))
    scen.startTimer()
    rospy.loginfo("Started node")
    rospy.spin()


if __name__ == '__main__':
    main()