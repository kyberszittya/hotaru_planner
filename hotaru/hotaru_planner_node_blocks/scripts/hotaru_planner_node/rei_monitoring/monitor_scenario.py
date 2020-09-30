import rospy

from autoware_msgs.msg import VehicleStatus

from jsk_rviz_plugins.msg import OverlayText

from rei_monitoring_msgs.msg import DetectedObstacles

class VehicleMonitor(object):

    def __init__(self):
        self.sub_vehicle_status = rospy.Subscriber("/vehicle_status", VehicleStatus, self.cb_vehicle_status)
        self.msg_vehicle_status = VehicleStatus()
        # Obstacle status
        self.sub_detected_obstacle = rospy.Subscriber("/detected_obstacles", DetectedObstacles, self.cb_detected_obstacles)
        self.msg_detected_obstacles = None
        self.pub_obstacle_text = rospy.Publisher("rei_vehicle_status/obstacle", OverlayText, queue_size=1)
        # Obstacle report
        self.msg_obstacle_text = OverlayText()
        self.msg_obstacle_text.text_size = 24
        self.msg_obstacle_text.width = 800
        self.msg_obstacle_text.height = 120
        self.msg_obstacle_text.bg_color.a = 0.5
        self.msg_obstacle_text.fg_color.r = 1.0
        self.msg_obstacle_text.fg_color.g = 0.0
        self.msg_obstacle_text.fg_color.b = 0.0
        self.msg_obstacle_text.fg_color.a = 1.0
        self.msg_obstacle_text.font = "Noto Mono"
        self.msg_obstacle_text.top = 500
        # State report
        self.msg_vehicle_state_text = OverlayText()
        self.msg_vehicle_state_text.text_size = 14
        self.msg_vehicle_state_text.width = 800
        self.msg_vehicle_state_text.height = 100
        self.msg_vehicle_state_text.bg_color.a = 0.5
        self.msg_vehicle_state_text.font = "Noto Mono"
        self.pub_vehicle_state_text = rospy.Publisher("rei_vehicle_status/state_text", OverlayText, queue_size=1)
        self.msg_vehicle_state_text.fg_color.r = 1.0
        self.msg_vehicle_state_text.fg_color.g = 0.0
        self.msg_vehicle_state_text.fg_color.b = 0.0
        self.msg_vehicle_state_text.fg_color.a = 1.0
        self.msg_vehicle_state_text.bg_color.a = 0.5
        # State report
        self.msg_state_text = OverlayText()
        self.msg_state_text.text_size = 14
        self.msg_state_text.width = 800
        self.msg_state_text.height = 100
        self.msg_state_text.bg_color.a = 0.5
        self.msg_state_text.font = "Noto Mono"



    def cb_vehicle_status(self, data):
        self.msg_vehicle_status = data
        kmph = data.linear_velocity*3.6
        self.msg_vehicle_state_text = ""
        self.msg_vehicle_state_text.fg_color.r = 0.0
        self.msg_vehicle_state_text.fg_color.g = 1.0
        self.msg_vehicle_state_text.fg_color.b = 0.0
        if kmph > 35.0:
            self.msg_vehicle_state_text.fg_color.r = 1.0
            self.msg_vehicle_state_text.fg_color.g = 0.0
            self.msg_vehicle_state_text.fg_color.b = 0.0
        deg_steer = data.angle * 3.14/180.0
        if deg_steer > 32.0:
            self.msg_vehicle_state_text.fg_color.r = 1.0
            self.msg_vehicle_state_text.fg_color.g = 0.0
            self.msg_vehicle_state_text.fg_color.b = 0.0
        self.msg_vehicle_state_text += "{0} km/h\n".format(kmph)
        self.msg_vehicle_state_text += "{0} deg".format(deg_steer)

    def cb_detected_obstacles(self, data):
        if len(data.obstacles) > 0:
            self.msg_obstacle_text.width = 800
            self.msg_obstacle_text.text = "OBSTACLE DETECTED ON LANE"
        else:
            self.msg_obstacle_text.text = ""
            self.msg_obstacle_text.width = 0

    def cb_viz_timer(self, event):
        self.pub_obstacle_text.publish(self.msg_obstacle_text)
        self.pub_vehicle_state_text.publish(self.msg_obstacle_text)

    def initialize_timers(self):
        self.timer_vizualization = rospy.Timer(rospy.Duration(0.05), self.cb_viz_timer)




def main():
    rospy.init_node("rei_vehicle_status_monitor")
    monitor = VehicleMonitor()
    monitor.initialize_timers()
    rospy.spin()

if __name__=="__main__":
    main()
