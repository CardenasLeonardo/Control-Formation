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

    No hay término Σⱼ. Los agentes actúan de forma completamente independiente
    respecto al agente 0 (que lleva la estructura virtual).
    """

    DT = 0.1

    def __init__(self, anchor_speed=0.5):
        self._anchor_speed = anchor_speed
        self._smooth_offset = {}   # {agent_index: (rx, ry)}

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

        if agent0_id not in neighbors_dict:
            return None

        x0, y0, theta_0 = neighbors_dict[agent0_id]

        desired = _compute_offsets(n_agents - 1, theta_0, angle_v, d)
        des_x, des_y = desired.get(agent_index, (0.0, 0.0))
        rix, riy = self._advance(agent_index, des_x, des_y)

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
