import math


def rotation_matrix(theta):
    """Matriz de rotación 2D."""
    c = math.cos(theta)
    s = math.sin(theta)
    return [[c, -s], [s, c]]


def rotate(R, v):
    """Aplica matriz de rotación R a vector v = [vx, vy]."""
    return [
        R[0][0] * v[0] + R[0][1] * v[1],
        R[1][0] * v[0] + R[1][1] * v[1]
    ]


def compute_offsets(n_followers, theta_leader, angle_v, d):
    """
    Calcula los offsets r[i] de la formación en V para cada seguidor.

    Parámetros:
      n_followers  : número de seguidores (N - 1)
      theta_leader : orientación actual del líder (radianes)
      angle_v      : ángulo de apertura de la V (radianes)
      d            : separación entre robots en el brazo (metros)

    Retorna:
      offsets : dict { follower_index: (rx, ry) }
                follower_index va de 1 a n_followers
    """

    n_left  = n_followers // 2
    n_right = n_followers - n_left   # el extra va a la derecha

    # Brazos izquierdo y derecho
    left_ids  = list(range(1, n_left + 1))
    right_ids = list(range(n_left + 1, n_followers + 1))

    offsets = {}

    R_left  = rotation_matrix(theta_leader + angle_v)
    R_right = rotation_matrix(theta_leader - angle_v)

    for k, idx in enumerate(left_ids, start=1):
        v = rotate(R_left, [-k * d, 0.0])
        offsets[idx] = (v[0], v[1])

    for k, idx in enumerate(right_ids, start=1):
        v = rotate(R_right, [-k * d, 0.0])
        offsets[idx] = (v[0], v[1])

    return offsets


class ConsensusFormation:

    def __init__(self, beta=1.5, k1=1.5, k2=1.5, vmax=1.0, wmax=1.0):
        """
        Implementa:
          ẋ_i = -Σ_{j∈N_i} a_ij (x_i - x_j) - β b_i^0 (x_i - x_0 - r_i)
        con a_ij=1 y ley de control polar para convertir el error a (v, w).

        Parámetros:
          beta : ganancia β del término de atracción al ancla (líder + offset)
          k1   : ganancia proporcional de la ley polar (velocidad lineal)
          k2   : ganancia angular de la ley polar
          vmax : saturación velocidad lineal
          wmax : saturación velocidad angular
        """

        self.beta  = beta
        self.k1    = k1
        self.k2    = k2
        self.vmax  = vmax
        self.wmax  = wmax

    def compute(self, state, neighbors, leader_state, offset, neighbor_offsets=None):
        """
        Implementa la ley de formación con consenso relativo:

          ẋ_i = -Σ_j a_ij [(x_i - r_i) - (x_j - r_j)]
                - β b_i^0 (x_i - x_0 - r_i)

        El consenso es sobre la posición desplazada (x_j - r_j), de modo que
        la formación deseada es punto de equilibrio exacto.

        state            : (x, y, theta)
        neighbors        : [(xj, yj), ...]  — vecinos (sin líder)
        leader_state     : (xL, yL) o None
        offset           : (rix, riy)        — offset de formación de este robot
        neighbor_offsets : [(rjx, rjy), ...] — offsets de cada vecino (mismo orden)
        """

        xi, yi, theta = state
        rix, riy = offset

        if neighbor_offsets is None:
            neighbor_offsets = [(0.0, 0.0)] * len(neighbors)

        ex = 0.0
        ey = 0.0

        # Consenso relativo: (x_j - r_j) - (x_i - r_i)
        for (xj, yj), (rjx, rjy) in zip(neighbors, neighbor_offsets):
            ex += (xj - rjx) - (xi - rix)
            ey += (yj - rjy) - (yi - riy)

        # Término del líder: β (x_0 + r_i - x_i)
        if leader_state is not None:
            xL, yL = leader_state
            ex += self.beta * ((xL + rix) - xi)
            ey += self.beta * ((yL + riy) - yi)

        # Ley de control polar
        a = math.sqrt(ex**2 + ey**2)

        if a < 1e-6:
            return 0.0, 0.0

        alpha_angle = math.atan2(ey, ex) - theta

        while alpha_angle >  math.pi: alpha_angle -= 2 * math.pi
        while alpha_angle < -math.pi: alpha_angle += 2 * math.pi

        v = self.k1 * a * math.cos(alpha_angle)
        w = self.k2 * alpha_angle + self.k1 * math.sin(alpha_angle) * math.cos(alpha_angle)

        v = max(-self.vmax, min(self.vmax, v))
        w = max(-self.wmax, min(self.wmax, w))

        return v, w