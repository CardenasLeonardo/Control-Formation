import math


class PolarControlLaw:
    """
    Ley de control polar para unicycle.

    forward_only=False (defecto):
        v = k1·a·cos(α)  →  ȧ = −k1·a·cos²(α) ≤ 0  (converge siempre, incluso en reversa)

    forward_only=True:
        v clampada a [0, vmax]. El robot gira en el lugar cuando |α| > π/2.
        Nunca reversa, más lento cerca de α = ±π.
    """

    def __init__(self, k1=1.0, k2=1.5, vmax=1.0, wmax=1.0, forward_only=False):
        self.k1           = k1
        self.k2           = k2
        self.vmax         = vmax
        self.wmax         = wmax
        self.forward_only = forward_only

    def compute(self, a, alpha):
        v_ref = self.k1 * a * math.cos(alpha)
        w_ref = self.k2 * alpha + self.k1 * math.sin(alpha) * math.cos(alpha)
        v_min = 0.0 if self.forward_only else -self.vmax
        v_ref = max(v_min,      min(self.vmax,  v_ref))
        w_ref = max(-self.wmax, min(self.wmax,  w_ref))
        return v_ref, w_ref
