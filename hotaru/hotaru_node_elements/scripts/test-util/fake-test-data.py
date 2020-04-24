'''
Created on Apr 24, 2020

@author: kyberszittya
'''

import rospy

from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, TwistStamped
from hotaru_msgs.msg import RefinedTrajectory
from rei_monitoring_msgs.msg import DetectedObstacles

import tf2_ros

class SimpleLocalPlannerTester(object):
    
    def __init__(self):
        self.pub_current_pose = rospy.Publisher("/current_pose", PoseStamped, queue_size=1)
        self.pub_current_velocity = rospy.Publisher("current_veloicty", TwistStamped, queue_size=1)
        self.pub_closest_waypoint = rospy.Publisher("/closest_waypoint", Int32, queue_size=1)
        self.pub_input_trajectory = rospy.Publisher("/input_trajectory", RefinedTrajectory, queue_size=1)
        self.pub_detected_obstacles = rospy.Publisher("/rei_perception_monitor/detected_obstacles", DetectedObstacles, queue_size=1)
        self.buffer = tf2_ros.Buffer()
        self.tf_publisher = tf2_ros.TransformBroadcaster(self.buffer)
        self.setStartConfiguration()
        # Start Timers
        self.timer_pose = rospy.Timer(rospy.Duration(1.0/40.0), self.cbTimerPose)
        self.timer_velocity = rospy.Timer(rospy.Duration(1.0/40.0), self.cbTimerPose)
        
        
    def setStartConfiguration(self):
        self.msg_pose = PoseStamped()
        self.msg_pose.header.frame_id = "map"
        self.msg_pose.pose.position.x = 0.0
        self.msg_pose.pose.position.y = 0.0
        self.msg_pose.pose.orientation.w = 0.0
        self.msg_velocity = TwistStamped()
        self.msg_velocity.header.frame_id = "map"
    
    def cbTimerPose(self, event):
        self.pub_current_pose.publish(self.msg_pose)
         
    def cbTimerVelocity(self, event):
        self.pub_current_velocity(self.msg_velocity)
        