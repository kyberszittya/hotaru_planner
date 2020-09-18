'''
Created on Sep 18, 2020

@author: kyberszittya
'''

import numpy as np

class VehicleModel(object):
    
    def __init__(self, max_angle):
        self.max_angle = max_angle
        self.min_angle = 