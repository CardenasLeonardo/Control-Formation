import math


def _compute_arc_offsets(n_followers, theta_leader, d, arc_span):
    """
    Arco: todos los seguidores a distancia d del líder,
    distribuidos uniformemente en un arco de arc_span radianes
    centrado detrás del líder (θ_L + π).

    Retorna dict {follower_index: (dx, dy)} en frame mundo.
    """
    offsets = {}

    if n_followers == 1:
        angle = theta_leader + math.pi
        offsets[1] = (d * math.cos(angle), d * math.sin(angle))
        return offsets

    for i in range(1, n_followers + 1):
        frac  = (i - 1) / (n_followers - 1)     # 0 … 1
        delta = (frac - 0.5) * arc_span          # −arc_span/2 … +arc_span/2
        angle = theta_leader + math.pi + delta
        offsets[i] = (d * math.cos(angle), d * math.sin(angle))

    return offsets


class ConsensusFormation2:

    def __init__(self):
        self._theta_L_smooth = None

    def _update_leader_angle(self, theta_L_raw, alpha=0.05):
        if self._theta_L_smooth is None:
            self._theta_L_smooth = theta_L_raw
        else:
            diff = theta_L_raw - self._theta_L_smooth
            if diff >  math.pi: diff -= 2 * math.pi
            if diff < -math.pi: diff += 2 * math.pi
            self._theta_L_smooth += alpha * diff
        return self._theta_L_smooth

    def step(self, state, neighbors_dict,
             leader_id, follower_index, n_followers,
             arc_span, d, beta):

        leader_state = None
        theta_L      = 0.0

        if leader_id in neighbors_dict:
            xL, yL, theta_L_raw = neighbors_dict[leader_id]
            leader_state = (xL, yL)
            theta_L = self._update_leader_angle(theta_L_raw)

        offsets_all = _compute_arc_offsets(n_followers, theta_L, d, arc_span)
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

        return self._compute(state, neighbors, leader_state,
                             my_offset, neighbor_offsets, beta)

    def _compute(self, state, neighbors, leader_state,
                 offset, neighbor_offsets, beta):
        """
        Ley de consenso relativo (idéntica a formación V):
          e_i = -Σ_j [(xi - ri) - (xj - rj)]  -  β[(xi - ri) - xL]
        """
        xi, yi, theta = state
        rix, riy = offset

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
