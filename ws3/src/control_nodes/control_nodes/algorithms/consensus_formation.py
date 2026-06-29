import math


def _compute_offsets(n_followers, theta_leader, angle_v, d):
    n_left  = n_followers // 2
    n_right = n_followers - n_left

    def _rot(theta):
        c, s = math.cos(theta), math.sin(theta)
        return [[c, -s], [s, c]]

    def _apply(R, vx, vy):
        return (R[0][0]*vx + R[0][1]*vy, R[1][0]*vx + R[1][1]*vy)

    R_left  = _rot(theta_leader + angle_v)
    R_right = _rot(theta_leader - angle_v)

    offsets = {}
    for k in range(1, n_left + 1):
        offsets[k] = _apply(R_left, -k * d, 0.0)
    for k in range(1, n_right + 1):
        offsets[n_left + k] = _apply(R_right, -k * d, 0.0)
    return offsets


class ConsensusFormation:
    """
    Formación en V con rate-limiting sobre la posición del ancla virtual.

    Cuando el líder gira, el ancla de cada seguidor no salta a la nueva posición
    instantáneamente: se desplaza hacia ella a un máximo de anchor_speed m/s.
    Esto evita que el destino aparezca "detrás" del robot y obligue a reversear.
    """

    DT = 0.1  # periodo del loop de control (s), debe coincidir con create_timer

    def __init__(self, anchor_speed=0.5):
        self._anchor_speed = anchor_speed
        self._smooth_offset = {}   # {follower_index: (rx, ry)}  frame mundo

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

    def step(self, state, neighbors_dict, leader_id, follower_index, n_followers, angle_v, d, beta):
        leader_state = None
        theta_L      = 0.0

        if leader_id in neighbors_dict:
            xL, yL, theta_L = neighbors_dict[leader_id]
            leader_state = (xL, yL)

        # Posiciones deseadas instantáneas (según ángulo actual del líder)
        desired = _compute_offsets(n_followers, theta_L, angle_v, d)

        # Avanzar cada ancla a velocidad limitada
        smooth = {i: self._advance(i, ox, oy) for i, (ox, oy) in desired.items()}

        my_offset = smooth.get(follower_index, (0.0, 0.0))

        neighbors        = []
        neighbor_offsets = []
        for rid, (xj, yj, _) in neighbors_dict.items():
            if rid == leader_id:
                continue
            try:
                j = int(rid.replace('robot', ''))
            except ValueError:
                continue
            neighbors.append((xj, yj))
            neighbor_offsets.append(smooth.get(j, (0.0, 0.0)))

        return self.compute(state, neighbors, leader_state, my_offset, neighbor_offsets, beta)

    def compute(self, state, neighbors, leader_state, offset, neighbor_offsets=None, beta=1.5):
        xi, yi, theta = state
        rix, riy = offset

        if neighbor_offsets is None:
            neighbor_offsets = [(0.0, 0.0)] * len(neighbors)

        ex, ey = 0.0, 0.0

        for (xj, yj), (rjx, rjy) in zip(neighbors, neighbor_offsets):
            ex += (xj - rjx) - (xi - rix)
            ey += (yj - rjy) - (yi - riy)

        if leader_state is not None:
            xL, yL = leader_state
            ex += beta * ((xL + rix) - xi)
            ey += beta * ((yL + riy) - yi)

        a = math.sqrt(ex**2 + ey**2)
        if a < 1e-6:
            return 0.0, 0.0

        alpha = math.atan2(ey, ex) - theta
        while alpha >  math.pi: alpha -= 2 * math.pi
        while alpha < -math.pi: alpha += 2 * math.pi

        return a, alpha
