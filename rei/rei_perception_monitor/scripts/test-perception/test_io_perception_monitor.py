'''
Created on Apr 23, 2020

@author: kyberszittya
'''

import rospy

from rei_monitoring_msgs.msg import DetectedObstacles

def cbDetectedObstacles(data):
    print(len(data.obstacles))

def main():
    rospy.init_node("test_io_perception_setup")
    sub_deetected_obstacles = rospy.Subscriber("/rei_perception_monitor/detected_obstacles", 
            DetectedObstacles, cbDetectedObstacles)
    rospy.spin()
    
if __name__=="__main__":
    main()   