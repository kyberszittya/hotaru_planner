#! /usr/bin/env python
'''
Created on May 12, 2020

@author: kyberszittya
'''


import rospy

from rei_monitoring_msgs.msg import ReiStateMachineTransitionSignal

import json
import rei_monitoring_msgs

SIGNALS = {
    "AllStateMessagesReceived": 1,
    "Replanning": 32
}

class TestCaseFactorySignal(object):
    
    def __init__(self):
        pass
    
    def loadTestCase(self, path):
        self.testcase = json.load(open(path))
        
    def generateTestCase(self):
        sigs = self.testcase["expected_signals"]
        signals = []
        for p in sigs:
            seq = []
            for s in p["sequence"]:
                seq.append(SIGNALS[s])
            signals.append([p["name"], seq])
        return signals

class MonitorSeq(object):
    
    def __init__(self, parent, topic_name, sig_seq):
        self.parent = parent
        self.topic_name = topic_name
        self.sig_seq = sig_seq
        self.sub = rospy.Subscriber(topic_name, ReiStateMachineTransitionSignal, self.cbSig)
        
    def cbSig(self, data):
        print(self.sig_seq)        
        if (len(self.sig_seq) > 0):
            s = self.sig_seq.pop(0)
            if (not data.sig_id == s):
                rospy.logerr("Unexpected signal received on {0}! SIG: {1}".format(self.topic_name, data.sig_id))
                self.parent.setInvalidTest()
        else:
            rospy.logerr("Unexpected signal received on {0}! SIG: {1}".format(self.topic_name, data.sig_id))
            self.parent.setInvalidTest()

class HybridSyncMonitor(object):
    
    def __init__(self):
        self.test_case = True
        self.subs = []
    
    def setInvalidTest(self):
        self.test_case = False
        
    def isTestValid(self):
        valid = True
        for sub in self.subs:
            valid = self.test_case and len(sub.sig_seq) == 0
        return valid
        
    def initEvaluation(self, signals):
        self.signals = signals
        for sig in self.signals:
            self.subs.append(MonitorSeq(self, sig[0], sig[1]))
        
        
def main():
    rospy.init_node("eval_sync_signal")
    testcase_factory = TestCaseFactorySignal()
    testcase_factory.loadTestCase(rospy.get_param("~test_case_path"))
    signals = testcase_factory.generateTestCase()
    hysyncmonitor = HybridSyncMonitor()
    hysyncmonitor.initEvaluation(signals)
    rospy.spin()
    if (hysyncmonitor.isTestValid()):
        rospy.loginfo("Test case successful, all signals received in order")
    else:
        rospy.logerr("Test case invalid")
    
if __name__=="__main__":
    main()