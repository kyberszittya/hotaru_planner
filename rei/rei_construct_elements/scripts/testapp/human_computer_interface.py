'''
Created on Sep 9, 2020

@author: kyberszittya
'''

import rospy
from rei_signal_msgs.msg import ReiRuntimeControl, ReiNotificationSignal, ReiTransitionEventSignal, ReiLocationEventSignal

import networkx

class HybridMachineHumanInterface(object):
    
    def __init__(self, hysm_name):
        self.hysm_name = hysm_name
        # Command publishers
        self.pub_control_signal = rospy.Publisher(self.hysm_name+"/control_signal", ReiRuntimeControl, queue_size=1)
        # Event subscribers
        self.sub_transition_events = rospy.Subscriber(self.hysm_name+"/transition_events", ReiTransitionEventSignal, self.cb_eventTransition)
        self.sub_location_events = rospy.Subscriber(self.hysm_name+"/location_events", ReiLocationEventSignal, self.cb_eventLocation)
        
    
    """
    @param data: Data to be handled 
    """    
    def cb_eventTransition(self, data):
        pass
    
    """
    @param data: Data to be handled
    """
    def cb_eventLocation(self, data):
        pass
        
    def issueEvent(self, event):
        msg = ReiRuntimeControl()
        msg.header.stamp = rospy.Time.now()
        msg.event_name = event
        self.pub_control_signal.publish(msg)
