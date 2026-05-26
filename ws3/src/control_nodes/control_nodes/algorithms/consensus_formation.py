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
    """
    Protocolo de consenso con anclaje al líder.

    Retorna el error polar (a, alpha) — magnitud y ángulo del error de consenso
    respecto a la orientación del robot. La ley de control y la saturación
    se aplican en capas separadas.
    """

    def __init__(self, beta=1.5):
        self.beta = beta

    def compute(self, state, neighbors, leader_state, offset, neighbor_offsets=None):
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
            ex += self.beta * ((xL + rix) - xi)
            ey += self.beta * ((yL + riy) - yi)

        a = math.sqrt(ex**2 + ey**2)

        if a < 1e-6:
            return 0.0, 0.0

        alpha = math.atan2(ey, ex) - theta
        while alpha >  math.pi: alpha -= 2 * math.pi
        while alpha < -math.pi: alpha += 2 * math.pi

        return a, alpha