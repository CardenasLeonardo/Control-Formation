import math
from math import atan2, sqrt


class ConsensusPromErr:
    """
    Protocolo de consenso por promedio de errores.

    Retorna el error polar (a, alpha) — magnitud y ángulo del error de consenso
    respecto a la orientación del robot.
    """

    def compute(self, state, neighbors):

        if len(neighbors) == 0:
            return 0.0, 0.0

        x, y, theta = state

        ex = sum(nx - x for nx, _ in neighbors) / len(neighbors)
        ey = sum(ny - y for _, ny in neighbors) / len(neighbors)

        a = sqrt(ex**2 + ey**2)

        if a < 1e-6:
            return 0.0, 0.0

        alpha = atan2(ey, ex) - theta
        while alpha >  math.pi: alpha -= 2 * math.pi
        while alpha < -math.pi: alpha += 2 * math.pi

        return a, alpha
