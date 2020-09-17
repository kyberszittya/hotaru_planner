'''
Created on Apr 24, 2020

@author: kyberszittya
'''

import rospy

from rei_monitoring_msgs.msg import ReiStateMachineTransitionSignal
from jsk_rviz_plugins.msg import OverlayText


class StateMachineVizualization(object):
    
    def __init__(self):
        self.prompt_text = "HOTARU_TEB_PLANNER_NODE: "
        self.pub_ov_text = rospy.Publisher("/rei_planner_text_viz", OverlayText, queue_size= 1)
        self.msg_text = OverlayText() 
        self.msg_text.font = "Monospace"
        self.msg_text.text_size = 24
        self.msg_text.width = 300
        self.msg_text.bg_color.a = 0.4
        self.msg_text.bg_color.r = 0.0
        self.msg_text.bg_color.g = 0.0
        self.msg_text.bg_color.b = 0.0
        # 
        self.msg_text.fg_color.a = 1.0
        self.sub_visualizer = rospy.Subscriber("/hotaru_planner/behav/sync_state_machine/current_state", ReiStateMachineTransitionSignal, self.cbTransitionSignal)
        self.pub_timer = rospy.Timer(rospy.Duration(0.1), self.cbTimerPublishText)
        
    def cbTimerPublishText(self, event):
        self.publishText()
    
    def setRelayText(self):
        self.msg_text.fg_color.r = 0.0
        self.msg_text.fg_color.g = 0.8
        self.msg_text.fg_color.b = 0.2
        self.msg_text.text = self.prompt_text+"RELAY" 
        
    def cbTransitionSignal(self, data):
        if data.sig_id == 32:
            self.msg_text.fg_color.r = 0.0
            self.msg_text.fg_color.g = 0.2
            self.msg_text.fg_color.b = 0.7
            self.msg_text.text = self.prompt_text+"REPLANNING"
        if data.sig_id == 33:
            self.setRelayText()
        self.publishText()
        
    def publishText(self):
        self.pub_ov_text.publish(self.msg_text)


def main():
    rospy.init_node("rei_planner_signal_visualizer")
    viz = StateMachineVizualization()
    viz.setRelayText()
    viz.publishText()
    rospy.loginfo("Vizualization started")
    rospy.spin()
    
    
if __name__=="__main__":
    main() 