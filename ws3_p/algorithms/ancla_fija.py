import math


class AnclaFija:
    """
    Controlador de estación para el robot pivote.

    En vez de seguir la estructura virtual giratoria, apunta siempre
    a una posición mundo FIJA (su posición inicial). Usa la misma
    transformación PPC que FormacionVSPPC para garantizar convergencia.

        ancla_mundo = (x0, y0)   ← constante
        eᵢ = β · PPC(ancla_mundo − xᵢ)

    Compatible con el mismo FollowerAgent/control_loop que los demás:
    acepta todos los kwargs de step() pero los ignora.
    """

    DT     = 0.1
    XI_MAX = 0.999

    def __init__(self, x0, y0, beta=2.0, rho_0=3.0, rho_inf=0.05, l_rate=0.04):
        self._x0, self._y0 = x0, y0
        self._beta  = beta
        self._rho_0   = rho_0
        self._rho_inf = rho_inf
        self._l_rate  = l_rate
        self._t       = 0.0

    def _rho(self):
        return (self._rho_0 - self._rho_inf) * math.exp(-self._l_rate * self._t) + self._rho_inf

    def step(self, state, neighbors_dict, **kwargs):
        self._t += self.DT

        xi, yi, theta = state
        ex = self._x0 - xi
        ey = self._y0 - yi

        a = math.sqrt(ex * ex + ey * ey)
        if a < 1e-9:
            return 0.0, 0.0

        # Transformación PPC
        rho  = self._rho()
        xi_e = min(a / rho, self.XI_MAX)
        a_T  = math.atanh(xi_e) * self._beta

        alpha = math.atan2(ey, ex) - theta
        while alpha >  math.pi: alpha -= 2 * math.pi
        while alpha < -math.pi: alpha += 2 * math.pi

        return a_T, alpha
