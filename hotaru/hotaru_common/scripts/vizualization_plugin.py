from  jsk_rviz_plugins.msg import OverlayText
from rei_monitoring_msgs.msg import ReiStateMachineTransitionSignal
from geometry_msgs.msg import TwistStamped

import rospy

class HotaruVisualizer(object):

    def cbStateTransitionLocalPlanner(self, data):
        self.state_overlay_msg.text = "hotaru_local_planner: "+data.transition_signal        
        if data.signal_name == "REPLANNING":
            self.state_overlay_msg.fg_color.r = 0.1
            self.state_overlay_msg.fg_color.g = 0.5
            self.state_overlay_msg.fg_color.b = 0.8
            self.state_overlay_msg.fg_color.a = 1
        elif data.signal_name == "RELAY":            
            self.state_overlay_msg.fg_color.r = 0
            self.state_overlay_msg.fg_color.g = 1
            self.state_overlay_msg.fg_color.b = 0
            self.state_overlay_msg.fg_color.a = 1
        self.pub_planner_state.publish(self.state_overlay_msg)

    def cbCurrentVelocity(self, data):
        self.velocity_overlay_msg.text = str(round(data.twist.linear.x*3.6,1))+" kmph"
        self.pub_overlay_text.publish(self.velocity_overlay_msg)


    def __init__(self):
        self.pub_planner_state = rospy.Publisher("/local_planner_state", OverlayText,  queue_size=10)
        self.state_overlay_msg = OverlayText()
        self.state_overlay_msg.bg_color.r = 0.0
        self.state_overlay_msg.bg_color.g = 0.0
        self.state_overlay_msg.bg_color.b = 0.0
        self.state_overlay_msg.bg_color.a = 0.2
        self.state_overlay_msg.width = 256
        self.state_overlay_msg.height = 100
        self.sub_state_local_planner = rospy.Subscriber("/hotaru_local_planner/state_transition", ReiStateTransition, 
            self.cbStateTransitionLocalPlanner, queue_size=10)
        # Velocity overlay
        self.pub_overlay_text = rospy.Publisher("/hud_current_velocity", OverlayText, queue_size=10)
        self.velocity_overlay_msg = OverlayText()
        self.velocity_overlay_msg.bg_color.r = 0.0
        self.velocity_overlay_msg.bg_color.g = 0.0
        self.velocity_overlay_msg.bg_color.b = 0.0
        self.velocity_overlay_msg.bg_color.a = 0.5
        self.velocity_overlay_msg.fg_color.r = 0.1
        self.velocity_overlay_msg.fg_color.g = 0.9
        self.velocity_overlay_msg.fg_color.b = 0.4
        self.velocity_overlay_msg.fg_color.a = 1
        self.velocity_overlay_msg.width = 256
        self.velocity_overlay_msg.height = 100
        self.velocity_overlay_msg.left = 700
        self.velocity_overlay_msg.top = 40
        self.velocity_overlay_msg.text_size = 24
        self.sub_state_local_planner = rospy.Subscriber("/current_velocity", TwistStamped, 
            self.cbCurrentVelocity, queue_size=10)
        


def main():
    rospy.init_node("hotaru_visualizer")
    hot = HotaruVisualizer()
    rospy.spin()

if __name__=="__main__":
    main()