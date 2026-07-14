import math


def _compute_offsets(n_agents, theta_vs, angle_v, d):
    n_left  = n_agents // 2
    n_right = n_agents - n_left

    def _rot(theta):
        c, s = math.cos(theta), math.sin(theta)
        return [[c, -s], [s, c]]

    def _apply(R, vx, vy):
        return (R[0][0]*vx + R[0][1]*vy, R[1][0]*vx + R[1][1]*vy)

    R_left  = _rot(theta_vs + angle_v)
    R_right = _rot(theta_vs - angle_v)

    offsets = {}
    for k in range(1, n_left + 1):
        offsets[k] = _apply(R_left,  -k * d, 0.0)
    for k in range(1, n_right + 1):
        offsets[n_left + k] = _apply(R_right, -k * d, 0.0)
    return offsets


class FormacionVSPPC:
    """
    Estructura virtual + Control de Rendimiento Prescrito (PPC).

    Envolvente de rendimiento:
        ρ(t) = (ρ₀ − ρ∞) · exp(−l · t) + ρ∞

    Garantiza ‖eᵢ‖ < ρ(t) para todo t ≥ 0 (si ‖eᵢ(0)‖ < ρ₀).

    Transformación del error:
        ξᵢ   = ‖eᵢ‖ / ρ(t)
        aᵢ_T = arctanh(ξᵢ)   →  ∞ cuando ξ → 1   (ganancia adaptativa)
        dirección preservada (alpha sin cambio)
    """

    DT     = 0.1
    XI_MAX = 0.999

    def __init__(self, anchor_speed=0.5, rho_0=3.0, rho_inf=0.10, l_rate=0.05):
        self._anchor_speed = anchor_speed
        self._rho_0        = rho_0
        self._rho_inf      = rho_inf
        self._l_rate       = l_rate
        self._t            = 0.0
        self._smooth_offset = {}

    def _rho(self):
        return (self._rho_0 - self._rho_inf) * math.exp(-self._l_rate * self._t) + self._rho_inf

    def _ppc_transform(self, ex, ey):
        a = math.sqrt(ex * ex + ey * ey)
        if a < 1e-9:
            return 0.0, 0.0
        rho = self._rho()
        xi  = min(a / rho, self.XI_MAX)
        a_T = math.atanh(xi)
        scale = a_T / a
        return ex * scale, ey * scale

    def _advance(self, i, desired_x, desired_y):
        max_step = self._anchor_speed * self.DT
        if i not in self._smooth_offset:
            self._smooth_offset[i] = (desired_x, desired_y)
            return desired_x, desired_y

        cx, cy = self._smooth_offset[i]
        dx, dy = desired_x - cx, desired_y - cy
        dist   = math.sqrt(dx * dx + dy * dy)

        if dist <= max_step:
            self._smooth_offset[i] = (desired_x, desired_y)
        else:
            cx += max_step * dx / dist
            cy += max_step * dy / dist
            self._smooth_offset[i] = (cx, cy)

        return self._smooth_offset[i]

    def rho_at(self, t=None):
        """Retorna ρ en el tiempo t (o en el tiempo interno actual)."""
        if t is None:
            return self._rho()
        return (self._rho_0 - self._rho_inf) * math.exp(-self._l_rate * t) + self._rho_inf

    def step(self, state, neighbors_dict, agent0_id, agent_index,
             n_agents, angle_v, d, beta):

        self._t += self.DT

        if agent0_id not in neighbors_dict:
            return None

        x0, y0, theta_0 = neighbors_dict[agent0_id]

        desired = _compute_offsets(n_agents - 1, theta_0, angle_v, d)
        des_x, des_y = desired.get(agent_index, (0.0, 0.0))
        rix, riy = self._advance(agent_index, des_x, des_y)

        xi, yi, theta = state

        ex_pos = (x0 + rix) - xi
        ey_pos = (y0 + riy) - yi

        ex_T, ey_T = self._ppc_transform(ex_pos, ey_pos)

        ex = beta * ex_T
        ey = beta * ey_T

        a = math.sqrt(ex * ex + ey * ey)
        if a < 1e-6:
            return 0.0, 0.0

        alpha = math.atan2(ey, ex) - theta
        while alpha >  math.pi: alpha -= 2 * math.pi
        while alpha < -math.pi: alpha += 2 * math.pi

        return a, alpha
