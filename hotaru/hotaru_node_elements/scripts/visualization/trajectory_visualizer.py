#! /usr/bin/env python
'''
Created on Apr 24, 2020

@author: kyberszittya
'''

import rospy

from hotaru_msgs.msg import RefinedTrajectory
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA





class TrajectoryVisualizer(object):

    
    def __init__(self):
        #self.input_trajectory_name = rospy.get_param("~/input_topic_name")
        self.input_trajectory_name = "input_trajectory"
        #self.output_trajectory_name = rospy.get_param("~/output_topic_name")
        self.output_trajectory_name = "refined_trajectory"
        # Colors
        self.refined_color_wp = ColorRGBA()
        self.refined_color_wp.r = 1.0
        self.refined_color_wp.g = 0.0
        self.refined_color_wp.b = 1.0
        self.refined_color_wp.a = 0.8 
        
        self.viz_output_trajectory = MarkerArray()        
        self.viz_input_trajectory = MarkerArray()
        
        self.pub_input_trajectory_viz = rospy.Publisher(self.input_trajectory_name+"_viz", MarkerArray, queue_size=1)
        self.pub_output_trajectory_viz = rospy.Publisher(self.output_trajectory_name+"_viz", MarkerArray, queue_size=1)
        self.sub_input_trajectory = rospy.Subscriber(self.input_trajectory_name, RefinedTrajectory, self.cbInputTrajectory)
        self.sub_output_trajectory = rospy.Subscriber(self.output_trajectory_name, RefinedTrajectory, self.cbOutputTrajectory)
        self.traj_pub_timer = rospy.Timer(rospy.Duration(0.1), self.cbVizTrajectory)
        

    def trajectoryToMarkers(self, traj, viz_marker, color_strip=None, color_wps=None):
        if (len(traj.waypoints) > 0):
            viz_marker.markers = []
            linestrip = Marker()
            linestrip.id = 0
            linestrip.type = Marker.LINE_STRIP
            linestrip.header.frame_id = 'map'
            if color_strip is None:
                linestrip.color.r = 0.1
                linestrip.color.g = 0.7
                linestrip.color.b = 0.8
                linestrip.color.a = 0.2
            else:
                linestrip.color = color_strip
            wpmarker = []
            for i,wp in enumerate(traj.waypoints):
                m = Marker()
                m.header.frame_id = 'map'
                m.id = i+1
                # Color
                if color_wps is None:
                    m.color.r = 0.0
                    m.color.g = 1.0
                    m.color.b = 1.0
                    m.color.a = 0.9
                else:
                    m.color = color_wps
                # Scale
                m.scale.x = 1.0
                m.scale.y = 0.1
                m.scale.z = 0.1
                # Pose
                m.pose = wp.pose.pose
                wpmarker.append(m)
                linestrip.points.append(wp.pose.pose.position)
            linestrip.pose = traj.waypoints[0].pose.pose
            linestrip.scale.x = 0.3
            viz_marker.markers.append(linestrip)
            viz_marker.markers.extend(wpmarker)
                
    
        
    def cbInputTrajectory(self, data):
        self.trajectoryToMarkers(data, self.viz_input_trajectory) 
        
        
    def cbOutputTrajectory(self, data):
        self.trajectoryToMarkers(data, self.viz_output_trajectory, color_wps=self.refined_color_wp)
        
    
    def cbVizTrajectory(self, event):
        self.pub_input_trajectory_viz.publish(self.viz_input_trajectory)
        self.pub_output_trajectory_viz.publish(self.viz_output_trajectory)

        

def main():
    rospy.init_node("trajectory_visualizer")
    viz = TrajectoryVisualizer()
    rospy.spin()
    
if __name__=="__main__":
    main()