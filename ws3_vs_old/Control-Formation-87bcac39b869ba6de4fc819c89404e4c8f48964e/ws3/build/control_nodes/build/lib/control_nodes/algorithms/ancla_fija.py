import math


class AnclaFija:
    """
    Controlador de estación para el robot pivote.

    Mantiene al robot en su posición de odometría inicial (0, 0) usando PPC.
    No depende de la estructura virtual giratoria: el ancla es siempre el
    origen del frame de odometría local (que coincide con la posición de
    spawn del robot en Gazebo).

        eᵢ = β · PPC((0,0) − (x,y)_odom)

    Compatible con el mismo control_loop que los demás consensus_types.
    """

    DT     = 0.1
    XI_MAX = 0.999

    def __init__(self, beta=2.0, rho_0=3.0, rho_inf=0.05, l_rate=0.04):
        self._beta    = beta
        self._rho_0   = rho_0
        self._rho_inf = rho_inf
        self._l_rate  = l_rate
        self._t       = 0.0

    def _rho(self):
        return (self._rho_0 - self._rho_inf) * math.exp(-self._l_rate * self._t) + self._rho_inf

    def step(self, state, neighbors_dict):
        self._t += self.DT

        xi, yi, theta = state

        # Ancla = origen del frame odom del robot (posición de spawn)
        ex = -xi
        ey = -yi

        a = math.sqrt(ex * ex + ey * ey)
        if a < 1e-9:
            return 0.0, 0.0

        rho  = self._rho()
        xi_e = min(a / rho, self.XI_MAX)
        a_T  = math.atanh(xi_e) * self._beta

        alpha = math.atan2(ey, ex) - theta
        while alpha >  math.pi: alpha -= 2 * math.pi
        while alpha < -math.pi: alpha += 2 * math.pi

        return a_T, alpha
