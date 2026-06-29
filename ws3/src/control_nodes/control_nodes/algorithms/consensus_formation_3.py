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


class ConsensusFormation3:
    """
    Formación en V con feedforward de velocidad del ancla virtual.

    Problema que resuelve (efecto de látigo):
      Cuando el líder gira a velocidad angular ω, el ancla del k-ésimo robot
      se mueve a velocidad lineal v_ancla = k·d·ω. Si v_ancla > vmax, el robot
      no puede seguirla, cae atrás y luego sobrecompensa — efecto látigo.

    Límite sin látigo:
      ω_max = vmax / (k_max · d)
      Ej: vmax=0.8, k_max=2, d=1.0 → ω_max = 0.4 rad/s

    Solución — feedforward de velocidad:
      En lugar de solo reaccionar al error de posición, el robot estima la
      velocidad instantánea de su ancla (diferencia finita) y agrega ese
      vector directamente al error:

        e_i = consenso_posicion + β·error_lider + γ·v_ancla_i

      Con γ adecuado, el robot anticipa el movimiento del ancla antes de que
      el error de posición se acumule → menos sobrecompensación → menos látigo.

    Parámetros nuevos vs ConsensusFormation:
      gamma       : ganancia del feedforward (default 0.3). Aumentar si hay látigo;
                    disminuir si hay oscilación.
      anchor_speed: rate-limit del ancla (m/s, default 0.5) — mismo mecanismo que V.
    """

    DT = 0.1  # debe coincidir con create_timer en consenso_node

    def __init__(self, anchor_speed=0.5, gamma=0.3):
        self._anchor_speed = anchor_speed
        self._gamma        = gamma
        self._smooth_offset = {}   # {i: (rx, ry)} posición suavizada del ancla
        self._prev_anchor   = {}   # {i: (rx, ry)} paso anterior (para diferencia finita)

    # ------------------------------------------------------------------
    # Rate-limiting de anclas (idéntico a ConsensusFormation)
    # ------------------------------------------------------------------

    def _advance(self, i, desired_x, desired_y):
        max_step = self._anchor_speed * self.DT
        if i not in self._smooth_offset:
            self._smooth_offset[i] = (desired_x, desired_y)
            self._prev_anchor[i]   = (desired_x, desired_y)
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

    # ------------------------------------------------------------------
    # Velocidad del ancla (diferencia finita sobre la posición suavizada)
    # ------------------------------------------------------------------

    def _anchor_velocity(self, i, current_x, current_y):
        if i not in self._prev_anchor:
            self._prev_anchor[i] = (current_x, current_y)
            return 0.0, 0.0
        px, py = self._prev_anchor[i]
        vx = (current_x - px) / self.DT
        vy = (current_y - py) / self.DT
        self._prev_anchor[i] = (current_x, current_y)
        return vx, vy

    # ------------------------------------------------------------------

    def step(self, state, neighbors_dict, leader_id, follower_index, n_followers, angle_v, d, beta):
        leader_state = None
        theta_L      = 0.0

        if leader_id in neighbors_dict:
            xL, yL, theta_L = neighbors_dict[leader_id]
            leader_state = (xL, yL)

        desired = _compute_offsets(n_followers, theta_L, angle_v, d)

        smooth = {}
        vel    = {}
        for i, (ox, oy) in desired.items():
            sx, sy   = self._advance(i, ox, oy)
            smooth[i] = (sx, sy)
            vel[i]    = self._anchor_velocity(i, sx, sy)

        my_offset   = smooth.get(follower_index, (0.0, 0.0))
        my_vel      = vel.get(follower_index,    (0.0, 0.0))

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

        return self._compute(state, neighbors, leader_state,
                             my_offset, my_vel, neighbor_offsets, beta)

    def _compute(self, state, neighbors, leader_state,
                 offset, anchor_vel, neighbor_offsets, beta):

        xi, yi, theta = state
        rix, riy = offset
        vax, vay = anchor_vel

        ex, ey = 0.0, 0.0

        # Término de consenso entre seguidores
        for (xj, yj), (rjx, rjy) in zip(neighbors, neighbor_offsets):
            ex += (xj - rjx) - (xi - rix)
            ey += (yj - rjy) - (yi - riy)

        # Término de atracción al líder
        if leader_state is not None:
            xL, yL = leader_state
            ex += beta * ((xL + rix) - xi)
            ey += beta * ((yL + riy) - yi)

        # Feedforward: anticipar el movimiento del ancla
        ex += self._gamma * vax
        ey += self._gamma * vay

        a = math.sqrt(ex**2 + ey**2)
        if a < 1e-6:
            return 0.0, 0.0

        alpha = math.atan2(ey, ex) - theta
        while alpha >  math.pi: alpha -= 2 * math.pi
        while alpha < -math.pi: alpha += 2 * math.pi

        return a, alpha
