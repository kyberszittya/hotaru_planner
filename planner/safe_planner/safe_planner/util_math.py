import math

def euclidean_distance_2d(state, goal):
    dx = state[0] - goal[0]
    dy = state[1] - goal[1]
    return math.sqrt(dx**2 + dy**2)