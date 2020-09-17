'''
Created on Apr 23, 2020

@author: kyberszittya
'''

import rospy
from rei_monitoring_msgs.msg import DetectedObstacles

from autoware_msgs.msg import DetectedObjectArray, DetectedObject
from geometry_msgs.msg import PoseStamped

from visualization_msgs.msg import MarkerArray, Marker

import tf2_ros
import tf2_geometry_msgs

import math
import json

OBSTACLE_PUBLISH_HZ = 10.0


def euclideanDistance(x0, y0, x1, y1):
    return math.sqrt((x0 - x1)**2+(y0 - y1)**2)




class FakeObjectInducer(object):
    
    def loadTestCase(self, filename):
        js = json.load(open(filename))
        self.obj_poses = []
        self.detected_obj = []
        self.marker_viz = []
        for obs in js["local_planner_test"]["obstacle"]:
            p = PoseStamped()
            p.pose.position.x = obs["x"]
            p.pose.position.y = obs["y"]
            self.obj_poses.append(p)
            self.detected_obj.append(DetectedObject())
            m = Marker()
            m.header.frame_id = self.sensor_frame
            self.marker_viz.append(m)
        
    
    def __init__(self):
        self.sensor_frame = "left_os1/os1_sensor"
        self.loadTestCase("./test-cases/test1.json")
        
        #
        self.pub_fake_object = rospy.Publisher("/detection/lidar_detector/objects", DetectedObjectArray, queue_size=10)
        self.sub_current_pose = rospy.Subscriber("/current_pose", PoseStamped, self.cbCurrentPose)
        # We assume circular obstacle
        self.msg_pub_fake = DetectedObjectArray()
        self.marker_array_viz = MarkerArray()        
        self.msg_pub_fake.header.frame_id = self.sensor_frame
        self.buffer = tf2_ros.Buffer(rospy.Duration(10.0))
        self.listener = tf2_ros.TransformListener(self.buffer)
        # Timer to publish
        
        self.obst_timer = rospy.Timer(rospy.Duration(1.0/OBSTACLE_PUBLISH_HZ), self.cbTimerDetectorObjectPublish)
        
    
    def cbCurrentPose(self, data):
        self.current_pose = data
    
    
    def cbTimerDetectorObjectPublish(self, event):
        transform = self.buffer.lookup_transform(self.sensor_frame, "map", rospy.Time(0), rospy.Duration(1.0))
        self.msg_pub_fake.objects = []
        self.marker_array_viz.markers = []
        for i,obst in enumerate(self.obj_poses):            
            d = euclideanDistance(obst.pose.position.x, obst.pose.position.y, 
                                  self.current_pose.pose.position.x, 
                                  self.current_pose.pose.position.y)
            
            if (d < 20.0):       
                print(d)
                self.detected_obj[i].pose = tf2_geometry_msgs.do_transform_pose(obst, transform).pose
                self.marker_viz[i].pose = self.detected_obj[i].pose
                self.msg_pub_fake.objects.append(self.detected_obj[i])
                self.marker_array_viz.markers.append(self.marker_viz[i])
        self.pub_fake_object.publish(self.msg_pub_fake)
        

def main():
    rospy.init_node("fake_object_inducer")
    fake_object_inducer = FakeObjectInducer()
    rospy.spin()
    

if __name__=="__main__":
    main()
     