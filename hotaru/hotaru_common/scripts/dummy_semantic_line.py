from geometry_msgs.msg import TwistStamped, PoseStamped, Point
from visualization_msgs.msg import MarkerArray, Marker

import rospy
import tf2_ros

MAP=[((639812.180312, 5195060.3411), (639812.180312, 5195000.3411)),
     ((639818.180312, 5195060.3411), (639818.180312, 5195000.3411)),
     ((639812.180312, 5195000.3411), (639782.180312, 5195000.3411)),
     ((639812.180312, 5194994.3411), (639782.180312, 5194994.3411)),
     ((639768.180372, 5195060.3411), (639768.180312, 5195000.3411)),
     ((639778.180372, 5195060.3411), (639778.180312, 5195000.3411))]


class SemanticLinePublisher(object):

    def __init__(self):
        self.sub_current_pose = rospy.Subscriber("/current_pose",PoseStamped, self.cbCurrentPose, queue_size=10)
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.semantic_obstacle_update = rospy.Timer(rospy.Duration(0.1), self.cbTimerObstacle)
        self.marker = MarkerArray()
        
        self.pub_marker_semantic_perimeter = rospy.Publisher("/map_semantic_line", MarkerArray, queue_size=1)

    def cbCurrentPose(self, data):
        self.transform = self.tf_buffer.lookup_transform("base_link",
            data.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
    
    def cbTimerObstacle(self, e):
        self.marker.markers = []
        for i,p in enumerate(MAP):
            m = Marker()
            m.id = i
            m.header.stamp = rospy.Time.now()
            m.header.frame_id = "map"
            m.type = Marker.LINE_LIST
            m.points.append(Point(p[0][0], p[0][1], 0.0))
            m.points.append(Point(p[1][0], p[1][1], 0.0))
            m.pose.orientation.w = 1.0            
            m.color.r = 0.0
            m.color.g = 1.0
            m.color.b = 1.0
            m.color.a = 1.0
            m.scale.x = 0.1
            m.scale.y = 0.1
            m.scale.z = 0.1
            self.marker.markers.append(m)
        self.pub_marker_semantic_perimeter.publish(self.marker)
        


def main():
    rospy.init_node("dummy_semantic_line")
    sem_line = SemanticLinePublisher()
    rospy.spin()

if __name__=="__main__":
    main()
