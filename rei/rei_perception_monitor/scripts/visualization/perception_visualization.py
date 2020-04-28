#! /usr/bin/env python
'''
Created on Apr 28, 2020

@author: kyberszittya
'''

import rospy

from rei_monitoring_msgs.msg import DetectedObstacles
from visualization_msgs.msg import MarkerArray, Marker

class PerceptionVisualizer(object):
    
    def __init__(self):
        self.msg_viz = MarkerArray()
        self.pub_obstacle_publisher = rospy.Publisher("/rei_perception/viz", MarkerArray, queue_size=1)        
        self.sub_detected_obstacles = rospy.Subscriber("/rei_perception_monitor/detected_obstacles", DetectedObstacles, self.cbDetectedObstacles)
        
        
    
    def cbDetectedObstacles(self, data):
        self.msg_viz.markers = []
        for o in data.obstacles:
            m = Marker()
            m.header.frame_id = data.header.frame_id
            m.header.stamp = data.header.stamp
            m.pose = o.pose
            m.type = Marker.SPHERE
            m.scale.x = 1.0
            m.scale.y = 1.0
            m.scale.z = 1.0
            
            m.color.r = 1.0
            m.color.g = 0.0
            m.color.b = 0.0
            m.color.a = 1.0
            
            self.msg_viz.markers.append(m)
        self.pub_obstacle_publisher.publish(self.msg_viz)

def main():
    rospy.init_node("perception_visualization")
    visualizer = PerceptionVisualizer()
    rospy.spin()
    
    
if __name__=="__main__":
    main()

