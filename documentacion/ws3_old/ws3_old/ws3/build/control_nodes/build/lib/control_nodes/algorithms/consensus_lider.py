import math
from math import atan2, sqrt, cos, sin
import numpy as np

class ConsensusLeader:

    def __init__(self, k1=0.8, k2=1.0, k_leader=10.0, vmax=1.0, wmax=1.0):

        self.k1       = k1
        self.k2       = k2
        self.k_leader = k_leader
        self.vmax     = vmax
        self.wmax     = wmax

    def compute(self, state, neighbors, leader_state=None):
        """
        state        = (x, y, theta)
        neighbors    = [(xj, yj), ...]
        leader_state = (xL, yL) o None
        """

        xi, yi, theta = state

        ex = 0.0
        ey = 0.0
        n  = 0.0

        # Influencia local de vecinos
        for xj, yj in neighbors:
            ex += (xj - xi)
            ey += (yj - yi)
            n  += 1.0

        if n == 0.0:
            return 0.0, 0.0

        # Influencia del líder si está disponible
        if leader_state is not None:
            xL, yL = leader_state
            ex += self.k_leader * (xL - xi)
            ey += self.k_leader * (yL - yi)
            n  += 1.0

           
        ex /= n
        ey /= n
          
        # -----------------------------
        # Ley de control polar
        # -----------------------------

        a     = sqrt(ex**2 + ey**2)
        alpha = atan2(ey, ex) - theta

        if alpha >  math.pi: alpha -= 2 * math.pi
        if alpha < -math.pi: alpha += 2 * math.pi

        v = self.k1 * a * cos(alpha)
        w = self.k2 * alpha + self.k1 * sin(alpha) * cos(alpha)

        v = max(-self.vmax, min(self.vmax, v))
        w = max(-self.wmax, min(self.wmax, w))

        return v, w