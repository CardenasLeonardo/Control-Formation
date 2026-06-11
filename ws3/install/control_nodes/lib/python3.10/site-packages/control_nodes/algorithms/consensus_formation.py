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

    def __init__(self):
        self._theta_L_smooth = None

    def _update_leader_angle(self, theta_L_raw, percentage=0.05):
        if self._theta_L_smooth is None:
            self._theta_L_smooth = theta_L_raw
        else:
            diff = theta_L_raw - self._theta_L_smooth
            if diff >  math.pi: diff -= 2 * math.pi
            if diff < -math.pi: diff += 2 * math.pi
            self._theta_L_smooth += percentage * diff
        return self._theta_L_smooth


    def step(self, state, neighbors_dict, leader_id, follower_index, n_followers, angle_v, d, beta):
        leader_state = None
        theta_L      = 0.0

        if leader_id in neighbors_dict:
            xL, yL, theta_L_raw = neighbors_dict[leader_id]
            leader_state = (xL, yL)
            theta_L = self._update_leader_angle(theta_L_raw)

        offsets_all = _compute_offsets(n_followers, theta_L, angle_v, d)
        my_offset   = offsets_all.get(follower_index, (0.0, 0.0))

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
            neighbor_offsets.append(offsets_all.get(j, (0.0, 0.0)))

        return self.compute(state, neighbors, leader_state, my_offset, neighbor_offsets, beta)



    def compute(self, state, neighbors, leader_state, offset, neighbor_offsets=None, beta=1.5):
        """
        Ley de consenso relativo:
          e_i = -Σ_j a_ij [(x_i - r_i) - (x_j - r_j)] - β b_i^0 (x_i - x_0 - r_i)

        Retorna (a, alpha):
          a     : magnitud del vector de error
          alpha : ángulo del error respecto al heading del robot (rad)
        """

        xi, yi, theta = state
        rix, riy = offset

        if neighbor_offsets is None:
            neighbor_offsets = [(0.0, 0.0)] * len(neighbors)

        ex = 0.0
        ey = 0.0

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