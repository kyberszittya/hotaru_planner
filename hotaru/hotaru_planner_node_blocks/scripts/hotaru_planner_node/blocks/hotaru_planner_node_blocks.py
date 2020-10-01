'''
Created on Sep 15, 2020

@author: kyberszittya
'''



class PlannerAlgorithm(object):
    
    def __init__(self):
        pass
    
    def plan(self):
        raise NotImplementedError
    
    def setState(self, start, goal):
        raise NotImplementedError
    