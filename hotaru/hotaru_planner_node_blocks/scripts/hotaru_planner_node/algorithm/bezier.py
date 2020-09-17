import numpy as np
from hotaru_planner_node.algorithm.hotaru_blocks import Interpolator

from scipy.special import comb

import matplotlib.pyplot as plt


def bernstein_polynomial(i, t, n):
    return float(comb(n, i)) * t**i * (1-t)**(n-i)


class BezierCurve(Interpolator):
    
    def __init__(self):
        Interpolator.__init__(self)
        self.maxT = 1
        
    def initialize_parameter_values(self):
        self.d_cv = {0: self.cvs}
        n_derivatives = 2
        for i in range(n_derivatives):
            n = len(self.d_cv[i])
            self.d_cv[i + 1] = np.array([(n - 1) * 
                (self.d_cv[i][j + 1] - self.d_cv[i][j]) for j in range(n - 1)])
        
        
    
    def weighting_function(self, t):
        return 1.0
    
    def calc_equation(self, i, t, cv):
        return bernstein_polynomial(i, t, self.n_points - 1) * cv.reshape(2,1)
    
    def _bezier(self, cv, t):
        rt = np.array([0, 0]).reshape(2,1)
        if t > self.maxT:
            return rt
        for i,cv in enumerate(self.cvs):
            rt = rt + self.calc_equation(i, t, cv)
        rt = rt * self.weighting_function(t)
        return rt
    
    def dr(self, t):
        return self._bezier(self.d_cv[1], t)
        
        
    def r(self, t):
        r = self._bezier(self.d_cv[0], t)
        dr = self._bezier(self.d_cv[1], t)
        dr /= np.linalg.norm(dr)
        ori = np.arctan2(dr[1],dr[0])
        return np.array([r[0], r[1], ori])


class RationalBezierCurve(BezierCurve):
    
    def __init__(self):
        BezierCurve.__init__(self)
        self.weights = []
        
    def set_weights(self, weights):
        self.weights = weights
        
    def weighting_function(self, t):
        weight = 0.0
        for i,w in enumerate(self.weights):
            weight += bernstein_polynomial(i, t, self.n_points - 1) * w
        return 1.0/weight

    def calc_equation(self, i, t, cv):
        return (bernstein_polynomial(i, t, self.n_points - 1) * cv.reshape(2,1) * self.weights[i])

