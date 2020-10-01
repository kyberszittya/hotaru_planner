'''
Created on Sep 15, 2020

@author: kyberszittya
'''

import numpy as np
import numpy.linalg as la

from collections import deque 

def dare_solve(A, B, Q, R, max_iter=150, epsilon=0.0001):
    x = Q
    x_next = Q
    
    for i in range(max_iter):
        _c_0 = A.T.dot(x).dot(A)
        _c_1 = A.T.dot(x).dot(B)
        _quot = R + B.T.dot(x).dot(B)
        if (_quot.shape[0]>1):
            _inv_R = la.inv(_quot) 
            x_next =  _c_0 - _c_1.dot(_inv_R).dot(B.T).dot(x).dot(A) + Q
            if (abs(x_next - x)).max() < epsilon:
                return x_next
        else:
            x_next = _c_0 - ((_c_1/_quot).dot(B.T.dot(x).dot(A))) + Q
            if (abs(x_next - x)).max() < epsilon:
                return x_next            
        x = x_next
    return x

class RobotSystemModel(object):
    
    def __init__(self, A, B):
        self.A = A
        self.B = B
        
    def getSystemModel(self):
        return self.A, self.B
    
class PositionalErrorSystemModel(RobotSystemModel):
    
    def __init__(self, dt):
        A = np.array([[dt, 1.0],
                      [0, dt]])
        B = np.array([[0.0, 1.0]]).reshape(2,1)
        RobotSystemModel.__init__(self, A, B)
        self.dt = dt


class LQRPlanner(object):
    
    def __init__(self, system_model,
                   goal_dist = 0.01, 
                   max_iter=150, 
                   max_time=30.0, epsilon=0.001):
        self.max_iter = max_iter
        self.max_time = max_time
        self.goal_dist = goal_dist
        self.epsilon = epsilon
        self.system_model = system_model
        # Store trajectory
        self.current_trajectory = deque()
    
    def dlqr(self, A, B, Q, R):
        X = dare_solve(A, B, Q, R, self.max_iter, self.epsilon)
        _quot = B.T.dot(X).dot(B) + R
        if _quot.shape[0] > 1:
            _c_inv = la.inv(_quot)
        else:
            _c_inv = 1/_quot
        
        K = _c_inv*(B.T.dot(X).dot(A))
        return K
    
    def lqr_control(self, A, B, x):
        Kopt = self.dlqr(A, B, np.eye(A.shape[0]), np.eye(1))
        u = -Kopt.dot(x)
        return u
    
    def get_trajectory(self):
        return self.current_trajectory
    
    def plan(self, start, goal):
        self.current_trajectory.clear()
        A, B = self.system_model.getSystemModel()
        time = 0.0
        self.current_trajectory.append(start)
        x = start - goal
        while time <= self.max_time:
            time += self.system_model.dt
            u = self.lqr_control(A,B,x)
            x = A.dot(x)  + B*u
            err = goal + x
            self.current_trajectory.append(err)         
            d = la.norm(goal - err)
            if d < self.goal_dist:
                break