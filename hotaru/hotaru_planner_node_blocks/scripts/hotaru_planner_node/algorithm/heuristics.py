'''
Created on Sep 17, 2020

@author: kyberszittya
'''

import numpy as np

def obstacle_avoidance_euclidean_distance(pr_x, pr_y, p0_x, p0_y, obstacle_radius=0.0,
        val_scale= 1.0, scale=1.0):
    """
    @param pr: Point of reference
    @param p0: Geometric point of interest
    @return: Geometric distance to the object, closest distance equals larger value
    """
    d = np.sqrt((pr_x - p0_x)**2 + (pr_y - p0_y)**2)
    d = np.sqrt(d**2 - obstacle_radius**2)
    return val_scale*np.exp(-d/(2*scale))

def euclidean_distance(pr_x, p0):
    return np.hypot(pr_x - p0)

def calc_cv_weights(pr, ps, func=lambda xr, yr, x0, y0: (xr-x0)**2 + (yr - y0)**2 ):
    weights = []
    for p in ps:
        weights.append(func(pr[0], pr[1], p[0], p[1]))
    return weights