import math
from math import atan2, sqrt, cos, sin

k1 = 0.8
k2 = 1.0

class ConsensusPromErr:

    def __init__(self, vmax=1.0, wmax=1.0):
        self.vmax = vmax
        self.wmax = wmax

    def compute(self, self_state, neighbors):
        """
        self_state = (x, y, theta)
        neighbors = [(xj, yj), ...]
        """

        if len(neighbors) == 0:
            return 0.0, 0.0

        x, y, theta = self_state

        suma_x = sum(nx - x for nx, _ in neighbors)
        suma_y = sum(ny - y for _, ny in neighbors)

        ex = suma_x / len(neighbors)
        ey = suma_y / len(neighbors)

        a = sqrt(ex**2 + ey**2)
        alpha = atan2(ey, ex) - theta

        # normalización , aprovechar el modulo que ya emple
        if alpha > math.pi:
            alpha -= 2 * math.pi
        if alpha < -math.pi:
            alpha += 2 * math.pi

        v = k1 * a * cos(alpha)
        w = k2 * alpha + k1 * sin(alpha) * cos(alpha)

        v = max(-self.vmax, min(self.vmax, v))
        w = max(-self.wmax, min(self.wmax, w))

        return v, w