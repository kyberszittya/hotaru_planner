import numpy as np

from hotaru_planner_node.algorithm.hotaru_blocks import Interpolator

def hermite_interpolation(p0, v0, t0, p1, v1, t1, t):
    dt = t - t0
    a0 = p0
    a1 = v0
    a2 = 3*(p1 - p0)/(t1 - t0)**2 - (v1 + 2 * v0)/(t1 - t0)
    a3 = 2*(p0 - p1)/(t1 - t0)**3 + (v1 + v0)/(t1 - t0)**2
    return a3*dt**3 + a2*dt**2 + a1*dt + a0

def d_hermite_interpolation(p0, v0, t0, p1, v1, t1, t):
    dt = t - t0
    a1 = v0
    a2 = 3*(p1 - p0)/(t1 - t0)**2 - (v1 + 2 * v0)/(t1 - t0)
    a3 = 2*(p0 - p1)/(t1 - t0)**3 + (v1 + v0)/(t1 - t0)**2
    return 3*a3*dt**2 + 2*a2*dt + a1


class CatmullRomSpline(Interpolator):

    def __init__(self, alpha=0.5, tension=0.5):
        Interpolator.__init__(self)
        # Parameters
        self.alpha = alpha
        self.tension = tension
        # Velocities
        self.ts = []
        self.velocities = []
        self.coefficients = None

    def initialize_parameter_values(self):
        ti = 0
        # Set time parameter
        self.maxT = 0
        self.ts.append(0)
        for i in range(1, len(self.cvs)):
            d = np.linalg.norm(self.cvs[i] - self.cvs[i - 1])
            ti += d**self.alpha + ti
            self.ts.append(ti)
            self.maxT = ti
        # Calc velocities
        for i in range(len(self.cvs)):
            self.velocities.append(self.calc_velocity(i))

    def calc_velocity(self, i):
        if i == 0:
            return 0.5 * (1 - self.tension) * (self.cvs[i + 1] - self.cvs[i])/(self.ts[i + 1] - self.ts[i])
        elif i == self.n_points - 1:
            return 0.5 * (1 - self.tension) * (self.cvs[i] - self.cvs[i - 1])/(self.ts[i] - self.ts[i - 1])
        else:
            return 0.5 * (1 - self.tension) * (self.cvs[i + 1] - self.cvs[i]) / (self.ts[i + 1] - self.ts[i]) + \
                0.5 * (1 - self.tension) * (self.cvs[i] - self.cvs[i - 1]) / (self.ts[i] - self.ts[i - 1])

    def dr(self, t, start=0):
        for i in range(start, len(self.cvs) - 1):
            if self.ts[i] <= t <= self.ts[i + 1]:
                p0 = self.cvs[i]
                v0 = self.velocities[i]
                t0 = self.ts[i]
                p1 = self.cvs[i + 1]
                v1 = self.velocities[i + 1]
                t1 = self.ts[i + 1]                
                dr = d_hermite_interpolation(p0, v0, t0, p1, v1, t1, t)
                dr = dr/np.linalg.norm(dr)
                return dr                
    
    def r(self, t, start=0):
        for i in range(start, len(self.cvs) - 1):
            if self.ts[i] <= t <= self.ts[i + 1]:
                p0 = self.cvs[i]
                v0 = self.velocities[i]
                t0 = self.ts[i]
                p1 = self.cvs[i + 1]
                v1 = self.velocities[i + 1]
                t1 = self.ts[i + 1]
                r = hermite_interpolation(p0, v0, t0, p1, v1, t1, t)
                return np.array([r[0], r[1]])

    def generate_uniform_path(self, slices):
        """
        Generate path

        :param steps: slices per section
        :return:
        """
        path = []
        dpath = []
        ts = []
        endp = False
        all_slices = 0
        for i in range(len(self.cvs) - 1):
            t0 = self.ts[i]
            t1 = self.ts[i + 1]

            if i == len(self.cvs) - 2:
                endp = True
            for t in np.linspace(t0, t1, slices, endpoint=endp):
                path.append(self.r(t, i))
                dpath.append(self.dr(t, i))
                ts.append(t)
                all_slices += 1
        return np.stack(path), np.stack(dpath), np.stack(ts), all_slices


