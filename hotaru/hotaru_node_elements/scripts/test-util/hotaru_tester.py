#! /usr/bin/env python
'''
Created on Apr 24, 2020

@author: kyberszittya
'''

import rospy

from std_msgs.msg import Int32
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped, TwistStamped
from hotaru_msgs.msg import RefinedTrajectory, Waypoint
from rei_monitoring_msgs.msg import DetectedObstacles, Obstacle
from rei_planner_signals.msg import ReplanRequest

import numpy as np

import tf2_ros

import json

# TODO: calculate closest waypoint to obstacle

class TestCaseFactory(object):
    
    def __init__(self):
        pass
        
    def loadTestCase(self, path):
        self.testcase = json.load(open(path))
        
    def generateTestCase(self):
        traj = self.testcase["reference_trajectory"]
        test_pose = self.testcase["state"]["pose"]
        vert = traj["vertices"]
        cv = []
        if traj["type"]=="POINT_TO_POINT":
            cv.append([vert[0][0], vert[0][1], vert[0][2]])
            cv.append([vert[1][0], vert[1][1], vert[1][2]])
            rospy.loginfo("Initializing test case point-to-point, with start state [{0},{1}] and end state [{2},{3}]"
                .format(cv[0][0], cv[0][1], cv[1][0], cv[1][1]))
        localplannertester = SimpleLocalPlannerTester()
        pose = [
            test_pose["position"]["x"],
            test_pose["position"]["y"],
            test_pose["position"]["z"],
            test_pose["orientation"]["x"],
            test_pose["orientation"]["y"],
            test_pose["orientation"]["z"],
            test_pose["orientation"]["w"]
        ]
        raw_obst =  self.testcase["obstacles"]
        obstacles = []
        for o in raw_obst:
            obstacles.append([
                o["position"]["x"], 
                o["position"]["y"], 
                o["position"]["z"],
                o["radius"]]
            )            
        localplannertester.initialize(cv, pose, obstacles)
        
        
        return localplannertester

class SimpleLocalPlannerTester(object):
    
    
    def __init__(self):
        self.pub_current_pose = rospy.Publisher("/current_pose", PoseStamped, queue_size=1)
        self.pub_current_velocity = rospy.Publisher("current_velocity", TwistStamped, queue_size=1)
        self.pub_closest_waypoint = rospy.Publisher("/closest_waypoint", Int32, queue_size=1)
        self.pub_input_trajectory = rospy.Publisher("/input_trajectory", RefinedTrajectory, queue_size=1)
        self.pub_detected_obstacles = rospy.Publisher("/rei_perception_monitor/detected_obstacles", DetectedObstacles, queue_size=1)
        self.pub_replan_sig = rospy.Publisher("replan_request_sig", ReplanRequest, queue_size=1)
        self.tf_publisher = tf2_ros.TransformBroadcaster()
    
        
    def initialize(self, cv, pose, obst):
        self.setStartConfiguration(pose)
        self.setStartTransform(pose)
        self.setTrajectoryMessage(cv)
        self.setObstacles(obst)
        
    
    def setTrajectoryMessage(self, cvertex):
        self.msg_input_trajectory = RefinedTrajectory()
        self.msg_input_trajectory.header.frame_id = "map"
        self.msg_closest_waypoint = Int32()
        self.msg_closest_waypoint.data = 0
        self.msg_detected_obst = DetectedObstacles()
        self.msg_replan_sig = ReplanRequest()
        
        if len(cvertex)==2:
            self.setLinearStartTrajectory(cvertex[0], cvertex[1])
        else:
            self.setInterpolatedTrajectory(cvertex)
            
    def setObstacles(self, obst):
        self.msg_detected_obst.header.frame_id = "base_link"
        if len(obst) > 0:
            for o in obst:
                op = Obstacle()                
                op.pose.position.x = o[0]
                op.pose.position.y = o[1]
                op.pose.position.z = o[2]
                op.radius = o[3]
                self.msg_detected_obst.obstacles.append(op)

        
    def start(self):
        # Start Timers
        self.timer_pose = rospy.Timer(rospy.Duration(1.0/40.0), self.cbTimerPose)
        self.timer_velocity = rospy.Timer(rospy.Duration(1.0/40.0), self.cbTimerVelocity)
        self.timer_trajectory = rospy.Timer(rospy.Duration(1.0/10.0), self.cbTrajectory)
        self.timer_closest_point = rospy.Timer(rospy.Duration(1.0/20.0), self.cbClosestPoint)
        self.timer_obstacle_detected = rospy.Timer(rospy.Duration(1.0/10.0), self.cbObstacleDetected)
        
        
    
    """
    E.g. 
    |       |  <- 40 meters length of trajectory 
    |       |
    |       |
    |       O  <- Obstacle at 15 meters
    |       |
    |_______|________________
    """
    def setLinearStartTrajectory(self, start=[0,0,0], end=[0,0,0], slices=40):
        rospy.loginfo("Setting linear trajectory between point: ({0},{1},{2})->({3},{4},{5})"
            .format(start[0], start[1], start[2], end[0], end[1], end[2]))
        t = (np.linspace(start[0], end[0], slices), np.linspace(start[1], end[1], slices))
        for i in range(slices - 1):
            wp = Waypoint()
            wp.pose.pose.position.x = t[0][i]
            wp.pose.pose.position.y = t[1][i]
            wp.pose.pose.orientation.w = 1.0
            self.msg_input_trajectory.waypoints.append(wp)        
    
     
    def cbClosestPoint(self, event):
        self.pub_closest_waypoint.publish(self.msg_closest_waypoint)
        
        
    def setStartConfiguration(self, pose):
        # Zero order state
        self.msg_pose = PoseStamped()
        self.msg_pose.header.frame_id = "map"
        self.msg_pose.pose.position.x = pose[0]
        self.msg_pose.pose.position.y = pose[1]
        self.msg_pose.pose.position.z = pose[2]
        self.msg_pose.pose.orientation.x = pose[3]
        self.msg_pose.pose.orientation.y = pose[4]
        self.msg_pose.pose.orientation.z = pose[5]
        self.msg_pose.pose.orientation.w = pose[6]
        # First order state
        self.msg_velocity = TwistStamped()
        self.msg_velocity.header.frame_id = "map"        
    
        
    def setStartTransform(self, pose):
        self.transform = TransformStamped()
        self.transform.header.frame_id = "map"
        self.transform.child_frame_id = "base_link"
        self.transform.transform.translation.x = pose[0]
        self.transform.transform.translation.y = pose[1]
        self.transform.transform.translation.z = pose[2]
        self.transform.transform.rotation.x = pose[3]
        self.transform.transform.rotation.y = pose[4]
        self.transform.transform.rotation.z = pose[5]
        self.transform.transform.rotation.w = pose[6]        
        
    
    def cbTimerPose(self, event):
        self.msg_pose.header.stamp = rospy.Time.now()
        self.pub_current_pose.publish(self.msg_pose)
        self.transform.header.stamp = rospy.Time.now()
        self.tf_publisher.sendTransform(self.transform)
         
    def cbTimerVelocity(self, event):
        self.msg_velocity.header.stamp = rospy.Time.now()
        self.pub_current_velocity.publish(self.msg_velocity)
        
    def cbTrajectory(self, event):
        self.msg_input_trajectory.header.stamp = rospy.Time.now()
        self.pub_input_trajectory.publish(self.msg_input_trajectory)
        
    def cbObstacleDetected(self, event):
        self.msg_detected_obst.header.stamp = rospy.Time.now()
        # TODO: this is losing edge
        self.msg_detected_obst.max_closest_waypoint = 10
        self.pub_detected_obstacles.publish(self.msg_detected_obst)
        if (len(self.msg_detected_obst.obstacles) > 0):
            self.msg_replan_sig.header.stamp = rospy.Time.now()
            self.msg_replan_sig.eval = True
            self.pub_replan_sig.publish(self.msg_replan_sig)
            

        
        
def main():
    rospy.init_node("local_planner_tester")
    testcase_factory = TestCaseFactory()
    testcase_factory.loadTestCase(rospy.get_param("~test_case_path"))
    localplannertest = testcase_factory.generateTestCase()
    localplannertest.start()
    rospy.spin()


if __name__=="__main__":
    main()