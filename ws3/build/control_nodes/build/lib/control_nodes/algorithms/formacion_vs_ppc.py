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

    La envolvente de rendimiento:
        ρ(t) = (ρ₀ − ρ∞) · exp(−l · t) + ρ∞

    garantiza que el error de posición ‖eᵢ‖ permanece estrictamente
    dentro de ρ(t) para todo t ≥ 0, siempre que la condición inicial
    se cumpla (‖eᵢ(0)‖ < ρ₀).

    Transformación del error (antes de la ley de control polar):
        ξᵢ   = ‖eᵢ‖ / ρ(t)            ∈ (0, 1)
        aᵢ_T = arctanh(ξᵢ)             → ∞ cuando ξ → 1
        dirección preservada → alpha no cambia

    Cuando el error se acerca a la cota, arctanh genera esfuerzo de
    control creciente que impide violarlo. Cuando el error es pequeño
    el comportamiento es casi lineal (arctanh(x) ≈ x para x << 1).

    Parámetros:
        rho_0       : cota inicial del error (m). Debe ser > error inicial.
        rho_inf     : precisión en estado estacionario (m).
        l_rate      : tasa de decaimiento de la envolvente (1/s).
        anchor_speed: rate-limit del ancla (m/s).
    """

    DT      = 0.1   # periodo del timer en consenso_node
    XI_MAX  = 0.999 # clamp de seguridad: evita arctanh(1) = inf

    def __init__(self, anchor_speed=0.5, rho_0=3.0, rho_inf=0.10, l_rate=0.05):
        self._anchor_speed = anchor_speed
        self._rho_0        = rho_0
        self._rho_inf      = rho_inf
        self._l_rate       = l_rate
        self._t            = 0.0           # tiempo desde inicio (s)
        self._smooth_offset = {}

    # ------------------------------------------------------------------
    # Envolvente de rendimiento
    # ------------------------------------------------------------------

    def _rho(self):
        return (self._rho_0 - self._rho_inf) * math.exp(-self._l_rate * self._t) + self._rho_inf

    # ------------------------------------------------------------------
    # Transformación PPC (solo magnitud; dirección intacta)
    # ------------------------------------------------------------------

    def _ppc_transform(self, ex, ey):
        """
        Recibe el error de posición (metros), retorna el error transformado.
        La dirección (atan2) no cambia. La magnitud pasa por arctanh(‖e‖/ρ).
        """
        a = math.sqrt(ex * ex + ey * ey)
        if a < 1e-9:
            return 0.0, 0.0

        rho = self._rho()
        xi  = min(a / rho, self.XI_MAX)   # clamp de seguridad
        a_T = math.atanh(xi)              # magnitud transformada

        scale = a_T / a
        return ex * scale, ey * scale

    # ------------------------------------------------------------------
    # Rate-limiting del ancla (idéntico al de FormacionVS)
    # ------------------------------------------------------------------

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

    # ------------------------------------------------------------------

    def step(self, state, neighbors_dict, agent0_id, agent_index,
             n_agents, angle_v, d, beta):

        self._t += self.DT   # avanzar reloj interno

        if agent0_id not in neighbors_dict:
            return None

        x0, y0, theta_0 = neighbors_dict[agent0_id]

        # Offset deseado con rate-limiting
        desired = _compute_offsets(n_agents - 1, theta_0, angle_v, d)
        des_x, des_y = desired.get(agent_index, (0.0, 0.0))
        rix, riy = self._advance(agent_index, des_x, des_y)

        xi, yi, theta = state

        # Error de posición puro (metros)
        ex_pos = (x0 + rix) - xi
        ey_pos = (y0 + riy) - yi

        # Transformación PPC
        ex_T, ey_T = self._ppc_transform(ex_pos, ey_pos)

        # Escalar por beta para la ley de control
        ex = beta * ex_T
        ey = beta * ey_T

        a = math.sqrt(ex * ex + ey * ey)
        if a < 1e-6:
            return 0.0, 0.0

        alpha = math.atan2(ey, ex) - theta
        while alpha >  math.pi: alpha -= 2 * math.pi
        while alpha < -math.pi: alpha += 2 * math.pi

        return a, alpha
