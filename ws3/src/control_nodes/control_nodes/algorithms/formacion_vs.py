import math


def _compute_offsets(n_agents, theta_vs, angle_v, d):
    """
    Offsets en frame mundo para n_agents agentes (sin incluir al agente 0).
    La estructura virtual tiene su origen en el agente 0 con orientación theta_vs.
    """
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


class FormacionVS:
    """
    Formación por estructura virtual pura — sin consenso inter-agente.

    Cada agente i sigue únicamente su ancla en la estructura virtual:

        eᵢ = β · (x₀ + rᵢ(θ₀) − xᵢ)

    No hay término Σⱼ. Los agentes no se coordinan entre sí;
    cada uno actúa de forma completamente independiente respecto al agente 0.

    El agente 0 lleva la estructura virtual: su posición es el origen de la VS
    y su orientación (θ₀) define la dirección de los brazos de la V.
    """

    DT = 0.1  # debe coincidir con el periodo del timer en consenso_node

    def __init__(self, anchor_speed=0.5):
        self._anchor_speed = anchor_speed
        self._smooth_offset = {}   # {agent_index: (rx, ry)} — ancla suavizada

    def _advance(self, i, desired_x, desired_y):
        max_step = self._anchor_speed * self.DT
        if i not in self._smooth_offset:
            self._smooth_offset[i] = (desired_x, desired_y)
            return desired_x, desired_y

        cx, cy = self._smooth_offset[i]
        dx, dy = desired_x - cx, desired_y - cy
        dist   = math.sqrt(dx*dx + dy*dy)

        if dist <= max_step:
            self._smooth_offset[i] = (desired_x, desired_y)
        else:
            cx += max_step * dx / dist
            cy += max_step * dy / dist
            self._smooth_offset[i] = (cx, cy)

        return self._smooth_offset[i]

    def step(self, state, neighbors_dict, agent0_id, agent_index,
             n_agents, angle_v, d, beta):
        """
        state         : (x, y, theta) del agente actual
        neighbors_dict: {robot_id: (x, y, theta)} — vecinos visibles
        agent0_id     : id del agente que lleva la estructura virtual
        agent_index   : índice de este agente en la formación (1, 2, ...)
        n_agents      : total de agentes (incluyendo el agente 0)
        """
        if agent0_id not in neighbors_dict:
            return None   # agente 0 no visible → detenerse

        x0, y0, theta_0 = neighbors_dict[agent0_id]

        # Offset deseado en frame mundo para este agente
        desired = _compute_offsets(n_agents - 1, theta_0, angle_v, d)
        des_x, des_y = desired.get(agent_index, (0.0, 0.0))

        # Rate-limiting: el ancla se mueve a máximo anchor_speed m/s
        rix, riy = self._advance(agent_index, des_x, des_y)

        # Error: solo ancla, sin consenso inter-agente
        xi, yi, theta = state
        ex = beta * ((x0 + rix) - xi)
        ey = beta * ((y0 + riy) - yi)

        a = math.sqrt(ex**2 + ey**2)
        if a < 1e-6:
            return 0.0, 0.0

        alpha = math.atan2(ey, ex) - theta
        while alpha >  math.pi: alpha -= 2 * math.pi
        while alpha < -math.pi: alpha += 2 * math.pi

        return a, alpha
