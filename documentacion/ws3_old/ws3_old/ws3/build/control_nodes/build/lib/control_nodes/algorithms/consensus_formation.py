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

    def __init__(self, alpha=0.01, beta=5, vmax=1.0, wmax=1.0):
        """
        Parámetros:
          alpha : ganancia del consenso entre vecinos
          beta  : ganancia de atracción al ancla virtual
          vmax  : saturación velocidad lineal
          wmax  : saturación velocidad angular
        """

        self.alpha = alpha
        self.beta  = beta
        self.vmax  = vmax
        self.wmax  = wmax

    def compute(self, state, neighbors, leader_state, offset):
        """
        Calcula las velocidades (v, w) para un seguidor.

        Parámetros:
          state        : (x, y, theta) del seguidor
          neighbors    : [(xj, yj), ...] — vecinos filtrados por AIRE
          leader_state : (xL, yL) — posición del líder
          offset       : (rx, ry) — offset de formación del seguidor

        Retorna:
          v, w : velocidades a aplicar
        """

        xi, yi, theta = state
        xL, yL = leader_state
        rx, ry = offset

        # --------------------------------------------------
        # Ancla virtual del seguidor
        # --------------------------------------------------

        x_ref = xL + rx
        y_ref = yL + ry

        # --------------------------------------------------
        # Error de consenso con vecinos
        # --------------------------------------------------

        ex = 0.0
        ey = 0.0

        for xj, yj in neighbors:
            ex += (xj - xi)
            ey += (yj - yi)

        n = max(len(neighbors), 1)
        ex /= n
        ey /= n

        # --------------------------------------------------
        # Error de atracción al ancla virtual
        # --------------------------------------------------

        ex += self.beta * (x_ref - xi)
        ey += self.beta * (y_ref - yi)

        # --------------------------------------------------
        # Ley de control polar
        # --------------------------------------------------

        a = math.sqrt(ex**2 + ey**2)

        if a < 1e-6:
            return 0.0, 0.0

        alpha_angle = math.atan2(ey, ex) - theta

        # Normalización del ángulo
        while alpha_angle >  math.pi: alpha_angle -= 2 * math.pi
        while alpha_angle < -math.pi: alpha_angle += 2 * math.pi

        v = 0.8 * a * math.cos(alpha_angle)
        w = 1.0 * alpha_angle + 0.8 * math.sin(alpha_angle) * math.cos(alpha_angle)

        # Saturación
        v = max(-self.vmax, min(self.vmax, v))
        w = max(-self.wmax, min(self.wmax, w))

        return v, w