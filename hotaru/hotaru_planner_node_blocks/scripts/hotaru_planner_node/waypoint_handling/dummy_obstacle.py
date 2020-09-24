from autoware_msgs.msg import CloudClusterArray, DetectedObjectArray, DetectedObject

from geometry_msgs.msg import Pose, PoseStamped

import tf2_ros
import tf2_geometry_msgs

import rospy


class DummyObstacleGenerator(object):

    def __init__(self, local_frame, global_frame, pose):
        # TF
        self.global_frame = global_frame
        self.local_frame = local_frame
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.trans = None
        #
        self.pub_dummy_obstacle = rospy.Publisher("detection/lidar_detector/objects", DetectedObjectArray, queue_size=1)
        self.obstacle_msg = DetectedObjectArray()
        self.obstacle_msg.header.frame_id = self.local_frame
        self.object = DetectedObject()
        self.pose = pose
        self.object.pose = pose.pose
        self.obstacle_msg.objects.append(self.object)

    def initialize_timers(self):
        self.timer_pub_object = rospy.Timer(rospy.Duration(0.05), self.cb_timer_object)

    def cb_timer_object(self, event):
        try:
            self.trans = self.tf_buffer.lookup_transform(self.local_frame, self.global_frame, rospy.Time())
            tr_pose = tf2_geometry_msgs.do_transform_pose(self.pose, self.trans)
            self.object.pose = tr_pose.pose
            self.object.header.stamp = rospy.Time.now()
            self.object.header.frame_id = self.local_frame
            self.pub_dummy_obstacle.publish(self.obstacle_msg)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("TF error")


def main():
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.x = 640190.6794
    pose.pose.position.y = 5193574.9518
    pose.pose.orientation.w = 1.0
    rospy.init_node("dummy_obstacle")
    obstacle_generator = DummyObstacleGenerator("base_link","map", pose)
    obstacle_generator.initialize_timers()
    rospy.spin()


if __name__=="__main__":
    main()