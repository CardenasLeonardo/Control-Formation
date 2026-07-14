import math
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from algorithms.control_law import PolarControlLaw


class CirculoNav:
    """
    Navegación online en circunferencia centrada en el pivote.

    Cada vez que el líder alcanza un waypoint, el siguiente se recalcula
    usando la posición ACTUAL del pivote (tomada de neighbors_dict).
    Así el líder siempre orbita alrededor de donde el pivote realmente está,
    no de su posición ideal pre-calculada.

    Parámetros
    ----------
    pivot_id  : str   — ID del robot pivote (ej. 'robot4')
    R         : float — radio de la circunferencia (distancia líder–pivote)
    n_circles : int   — número de vueltas completas
    turn_dir  : +1 (CCW) / -1 (CW)
    chord     : float — longitud de cuerda objetivo entre waypoints (m)
    """

    ARRIVE_DIST = 0.20

    def __init__(self, pivot_id, R, n_circles=2, turn_dir=+1,
                 chord=0.25, vmax=0.5, wmax=0.6, k1=1.0, k2=1.5):
        self._pivot_id  = pivot_id
        self._R         = R
        self._turn_dir  = turn_dir
        self._total_phi = 2 * math.pi * n_circles
        self._phi_done  = 0.0

        # Δφ tal que la cuerda ≈ chord: chord = 2R·sin(Δφ/2)
        self._dphi = 2 * math.asin(min(chord / (2 * R), 1.0))

        self._wp      = None   # waypoint actual (x, y)
        self._phi_wp  = None   # ángulo del waypoint actual respecto al pivote

        self._law = PolarControlLaw(k1=k1, k2=k2, vmax=vmax, wmax=wmax)
        self.done = False

    def _advance(self, px, py, phi_from):
        """Calcula el siguiente punto en la circunferencia Δφ adelante."""
        phi_next = phi_from + self._turn_dir * self._dphi
        return (px + self._R * math.cos(phi_next),
                py + self._R * math.sin(phi_next),
                phi_next)

    def step(self, state, neighbors_dict=None):
        if self.done:
            return 0.0, 0.0

        x, y, theta = state

        if neighbors_dict is None or self._pivot_id not in neighbors_dict:
            return 0.0, 0.0

        px, py, _ = neighbors_dict[self._pivot_id]

        # Primer waypoint: Δφ adelante desde la posición inicial del líder
        if self._wp is None:
            phi0 = math.atan2(y - py, x - px)
            wx, wy, phi_next = self._advance(px, py, phi0)
            self._wp     = (wx, wy)
            self._phi_wp = phi_next

        xg, yg = self._wp
        dist = math.hypot(xg - x, yg - y)

        if dist < self.ARRIVE_DIST:
            self._phi_done += self._dphi

            if self._phi_done >= self._total_phi:
                self.done = True
                return 0.0, 0.0

            # Siguiente waypoint usando la posición ACTUAL del pivote
            wx, wy, phi_next = self._advance(px, py, self._phi_wp)
            self._wp     = (wx, wy)
            self._phi_wp = phi_next

            xg, yg = self._wp
            dist = math.hypot(xg - x, yg - y)

        alpha = math.atan2(yg - y, xg - x) - theta
        while alpha >  math.pi: alpha -= 2 * math.pi
        while alpha < -math.pi: alpha += 2 * math.pi

        return self._law.compute(dist, alpha)

    @property
    def current_waypoint(self):
        return self._wp
