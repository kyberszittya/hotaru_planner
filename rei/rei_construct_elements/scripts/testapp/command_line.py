'''
Created on Sep 9, 2020

@author: kyberszittya
'''

import rospy
from rei_signal_msgs.msg import ReiRuntimeControl, ReiNotificationSignal 

from human_computer_interface import HybridMachineHumanInterface
        
class HybridSystemHumanMachineInterface(object):
    
    def __init__(self):
        self.hysm = []
        
    def addHySm(self, hy):
        self.hysm.append(hy)
        
class HybridHumanCommandLine(HybridMachineHumanInterface):
    
    def __init__(self, hysm_name):
        HybridMachineHumanInterface.__init__(self, hysm_name)
        
    def cb_eventTransition(self, data):
        HybridMachineHumanInterface.cb_eventTransition(self, data)
        print("({0}): {1}, {2}->{3}".format(
            self.hysm_name, data.event_name, data.source_location, data.target_location))
    
    def cb_eventLocation(self, data):
        HybridMachineHumanInterface.cb_eventLocation(self, data)
        print("({0}): {1} @ {2}".format(self.hysm_name, data.event_name, data.location))

def main():
    rospy.init_node("command_line_tester")
    hysm_name = raw_input("Enter name of the hybrid SM: ")
    h1 = HybridHumanCommandLine(hysm_name)
    hy_system = HybridSystemHumanMachineInterface()
    hy_system.addHySm(h1)
    print("Initialized command line tester")
    while(not rospy.is_shutdown()):
        event_name = raw_input("Next event: ")
        h1.issueEvent(event_name)
    
if __name__=="__main__":
    main()
    
    