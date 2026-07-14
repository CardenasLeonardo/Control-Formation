import math
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from algorithms.control_law import PolarControlLaw


class WaypointsNav:
    """
    Navega secuencialmente por una lista de waypoints usando la ley de control polar.
    Equivale a navigate_waypoints_pva sin el módulo PVA (sin obstáculos).
    """

    ARRIVE_DIST = 0.20  # umbral de llegada (m)

    def __init__(self, waypoints_flat, vmax=0.5, wmax=0.6, k1=1.0, k2=1.5):
        pts = list(waypoints_flat)
        self._wps = [(pts[i], pts[i + 1]) for i in range(0, len(pts) - 1, 2)]
        self._idx = 0
        self._law = PolarControlLaw(k1=k1, k2=k2, vmax=vmax, wmax=wmax)
        self.done = False

    def step(self, state, neighbors_dict=None):
        """Retorna (v, w). Se detiene cuando todos los waypoints son visitados."""
        if self.done:
            return 0.0, 0.0

        x, y, theta = state
        xg, yg = self._wps[self._idx]

        a = math.hypot(xg - x, yg - y)
        if a < self.ARRIVE_DIST:
            self._idx += 1
            if self._idx >= len(self._wps):
                self.done = True
                return 0.0, 0.0
            xg, yg = self._wps[self._idx]
            a = math.hypot(xg - x, yg - y)

        alpha = math.atan2(yg - y, xg - x) - theta
        while alpha >  math.pi: alpha -= 2 * math.pi
        while alpha < -math.pi: alpha += 2 * math.pi

        return self._law.compute(a, alpha)

    @property
    def current_waypoint(self):
        if self._idx < len(self._wps):
            return self._wps[self._idx]
        return None
