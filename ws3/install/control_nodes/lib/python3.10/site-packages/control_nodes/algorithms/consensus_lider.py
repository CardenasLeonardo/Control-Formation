import math
from math import atan2, sqrt, cos, sin

k1 = 0.8
k2 = 1.0


class ConsensusLeader:

    def __init__(self, k_leader=1.5, vmax=1.0, wmax=1.0):

        self.k_leader = k_leader
        self.vmax = vmax
        self.wmax = wmax

    def compute(self, state, neighbors, leader_state=None):
        """
        state        = (x, y, theta)
        neighbors    = [(xj, yj), ...]
        leader_state = (xL, yL)
        """

        xi, yi, theta = state

        # Sin líder no hay acción
        if leader_state is None:
            return 0.0, 0.0

        xL, yL = leader_state

        # -----------------------------
        # Error directo al líder
        # -----------------------------

        ex = xL - xi
        ey = yL - yi

        # -----------------------------
        # Ley de control polar
        # -----------------------------

        a     = sqrt(ex**2 + ey**2)
        alpha = atan2(ey, ex) - theta

        if alpha >  math.pi: alpha -= 2 * math.pi
        if alpha < -math.pi: alpha += 2 * math.pi

        v = k1 * a * cos(alpha)
        w = k2 * alpha + k1 * sin(alpha) * cos(alpha)

        v = max(-self.vmax, min(self.vmax, v))
        w = max(-self.wmax, min(self.wmax, w))

        return v, w