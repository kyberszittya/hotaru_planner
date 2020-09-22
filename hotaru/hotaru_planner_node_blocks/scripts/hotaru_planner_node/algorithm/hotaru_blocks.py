'''
Created on Sep 16, 2020

@author: kyberszittya
'''

import numpy as np



def sparse_points(tr, epsilon=1.0):
    selected_indices = set([len(tr)-1])
    deselected_indices = set()
    for i in range(len(tr)):
        if i in deselected_indices:
            continue
        selected_indices.add(i)
        for j in range(i + 1, len(tr)):
            d = np.linalg.norm(tr[i] - tr[j])
            if d < epsilon and j not in selected_indices:
                deselected_indices.add(j)
    print(deselected_indices)
    print(selected_indices)
    return tr[list(selected_indices)]



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
    
    def r(self, t, start=0):
        raise NotImplementedError
    
    def dr(self, t, start=0):
        raise NotImplementedError
    
    def ddr(self,t , start=0):
        raise NotImplementedError
    
    def generate_dpath(self, steps):
        """
        Return with derviative in t
        """
        path = []
        for t in np.linspace(0, self.maxT, steps):
            path.append(self.dr(t))
        return np.stack(path)
        
    def generate_path(self, steps):
        """
        Return with yaw (axis-z rotation)
        """
        path = []
        for t in np.linspace(0, self.maxT, steps):
            path.append(self.r(t))
        return np.stack(path)


class PlannerNode(object):

    def __init__(self, algorithm, interpolator):
        self.algorithm = algorithm
        self.interpolator = interpolator


    def calc_plan(self, start, goal):
        self.algorithm.plan(start, goal)



    def generate_path(self):
        t = self.algorithm.get_trajectory()
        st = sparse_points(np.stack(t))
        self.interpolator.add_control_vertices(st)
        self.interpolator.initialize_parameter_values()
        return self.interpolator.generate_path(100)

