import math


# ------------------------------------------------------------------
# Geometría de la formación en V
# ------------------------------------------------------------------

def _compute_offsets(n_robots, theta_v, angle_v, d):
    # Devuelve {robot_index: (rx, ry)} para todos los robots.
    # Índice 0 queda en el centroide virtual (offset nulo).
    n_followers = n_robots - 1
    n_left  = n_followers // 2
    n_right = n_followers - n_left

    def _rot(theta):
        c, s = math.cos(theta), math.sin(theta)
        return [[c, -s], [s, c]]

    def _apply(R, vx, vy):
        return (R[0][0]*vx + R[0][1]*vy, R[1][0]*vx + R[1][1]*vy)

    R_left  = _rot(theta_v + angle_v)
    R_right = _rot(theta_v - angle_v)

    offsets = {0: (0.0, 0.0)}
    for k in range(1, n_left + 1):
        offsets[k] = _apply(R_left,  -k * d, 0.0)
    for k in range(1, n_right + 1):
        offsets[n_left + k] = _apply(R_right, -k * d, 0.0)
    return offsets


# ------------------------------------------------------------------
# Algoritmo de consenso
# ------------------------------------------------------------------

class ConsensusFormation:
    # Formación en V sin suavizado: los offsets siguen instantáneamente
    # el heading de la estructura virtual.

    def step(self, state, neighbors_dict, virtual_state, follower_index, n_robots, angle_v, d, beta):

        # --- Estado del centroide virtual ---
        leader_state = None
        theta_v      = 0.0
        if virtual_state is not None:
            x_v, y_v, theta_v = virtual_state
            leader_state = (x_v, y_v)

        # --- Calcular offsets de la V ---
        offsets   = _compute_offsets(n_robots, theta_v, angle_v, d)
        my_offset = offsets.get(follower_index, (0.0, 0.0))

        # --- Recolectar vecinos físicos y sus offsets ---
        neighbors        = []
        neighbor_offsets = []
        for rid, (xj, yj, _) in neighbors_dict.items():
            try:
                j = int(rid.replace('robot', ''))
            except ValueError:
                continue
            neighbors.append((xj, yj))
            neighbor_offsets.append(offsets.get(j, (0.0, 0.0)))

        return self._compute(state, neighbors, leader_state, my_offset, neighbor_offsets, beta)

    def _compute(self, state, neighbors, leader_state, offset, neighbor_offsets, beta):

        xi, yi, theta = state
        rix, riy      = offset

        # --- Error de formación: suma de errores relativos en posición corregida ---
        ex, ey = 0.0, 0.0
        for (xj, yj), (rjx, rjy) in zip(neighbors, neighbor_offsets):
            ex += (xj - rjx) - (xi - rix)
            ey += (yj - rjy) - (yi - riy)

        # --- Atracción al centroide virtual (ponderada por beta) ---
        if leader_state is not None:
            xL, yL = leader_state
            ex += beta * ((xL + rix) - xi)
            ey += beta * ((yL + riy) - yi)

        # --- Magnitud y ángulo del error → entrada a la ley de control ---
        a = math.sqrt(ex**2 + ey**2)
        if a < 1e-6:
            return 0.0, 0.0

        alpha = math.atan2(ey, ex) - theta
        while alpha >  math.pi: alpha -= 2 * math.pi
        while alpha < -math.pi: alpha += 2 * math.pi

        return a, alpha
