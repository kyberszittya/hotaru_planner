import numpy

from dynamic_lane_astar import PolygonRepresentation

import numpy as np

def ex_polygon_inside():
    poly = PolygonRepresentation()
    v0 = np.array([-2.0, -2.0])
    v1 = np.array([-2.0, 2.0])
    v2 = np.array([2.0, 2.0])
    v3 = np.array([2.0, -2.0])
    p0 = np.array([0.0, 0.0])

    print(poly.inside_polygon(v0, v1, v2, v3, p0))
    p1 = np.array([0.0, 5.0])
    print(poly.inside_polygon(v0, v1, v2, v3, p1))
    p2 = np.array([1.0, 1.0])
    print(poly.inside_polygon(v0, v1, v2, v3, p2))

if __name__=="__main__":
    ex_polygon_inside()