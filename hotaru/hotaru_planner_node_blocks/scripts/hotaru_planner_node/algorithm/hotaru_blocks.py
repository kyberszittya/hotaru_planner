'''
Created on Sep 16, 2020

@author: kyberszittya
'''

import numpy as np

class Interpolator(object):
    
    def __init__(self):
        self.cvs = []
        self.maxT = 0.0
        self.n_points = 0
    
    def add_control_vertices(self, cv):
        self.cvs = cv
        self.n_points = len(self.cvs)
        
    def initialize_parameter_values(self):
        raise NotImplementedError
    
    def r(self, t):
        raise NotImplementedError
        
    def generate_path(self, steps):
        path = []
        for t in np.linspace(0, self.maxT, steps):
            path.append(self.r(t))
        return np.stack(path)
            