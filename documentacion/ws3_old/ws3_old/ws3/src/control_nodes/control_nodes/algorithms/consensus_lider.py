import math
from math import atan2, sqrt


class ConsensusLeader:
    """
    Protocolo de consenso con seguimiento de líder.

    Retorna el error polar (a, alpha) — magnitud y ángulo del error de consenso
    respecto a la orientación del robot.
    """

    def __init__(self, k_leader=10.0):
        self.k_leader = k_leader

    def compute(self, state, neighbors, leader_state=None):

        xi, yi, theta = state

        ex = 0.0
        ey = 0.0
        n  = 0.0

        for xj, yj in neighbors:
            ex += (xj - xi)
            ey += (yj - yi)
            n  += 1.0

        if n > 0:
            ex /= n
            ey /= n

        has_leader = leader_state is not None
        if has_leader:
            xL, yL = leader_state
            ex += self.k_leader * (xL - xi)
            ey += self.k_leader * (yL - yi)

        if n == 0.0 and not has_leader:
            return 0.0, 0.0

        a = sqrt(ex**2 + ey**2)

        if a < 1e-6:
            return 0.0, 0.0

        alpha = atan2(ey, ex) - theta
        while alpha >  math.pi: alpha -= 2 * math.pi
        while alpha < -math.pi: alpha += 2 * math.pi

        return a, alpha
