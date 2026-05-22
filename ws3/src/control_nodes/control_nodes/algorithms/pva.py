import numpy as np
from scipy.optimize import minimize


class PVA:

    def __init__(self,
                 d_safe=0.3,
                 d_influence=1.0,
                 xi=1.0,
                 rp=0.25,
                 v_max=0.5,
                 w_max=0.5,
                 angle_min=-np.pi,
                 angle_increment=np.pi/180,
                 n_rays=360,
                 deadlock_tol=0.05):

        self.d_safe = d_safe
        self.d_influence = d_influence
        self.xi = xi

        self.v_max = v_max
        self.w_max = w_max
        self.deadlock_tol = deadlock_tol

        # Precalcular A y B
        angles = angle_min + np.arange(n_rays) * angle_increment

        self.A = np.cos(angles)
        self.B = rp * np.sin(angles)

        # -----------------------------
        # Estado del boundary following
        # -----------------------------

        self.mode = 'reaching_goal'   # 'reaching_goal' | 'boundary_following'
        self.V_block = None           # V(z) registrada en el deadlock
        self.search_direction = None  # 'cw' | 'ccw'
        self.constraint_to_follow = None  # (A, B, C) de la constraint bloqueante

    # ---------------------------------------------------------
    # BUILD CONSTRAINTS
    # ---------------------------------------------------------

    def build_constraints(self, ranges):

        d = np.asarray(ranges)

        mask = (d > 0) & (d <= self.d_influence)

        if not np.any(mask):
            return []

        d = d[mask]

        ratio = (d - self.d_safe) / (self.d_influence - self.d_safe)

        C = self.xi * ratio

        A = self.A[mask]
        B = self.B[mask]

        constraints = list(zip(A, B, C))

        return constraints

    # ---------------------------------------------------------
    # SOLVE QP — módulo 1 (reaching the goal)
    # ---------------------------------------------------------

    def solve_qp(self, v_goal, w_goal, constraints):

        def objective(x):
            v, w = x
            return 0.5 * ((v - v_goal)**2 + (w - w_goal)**2)

        cons = []

        for A, B, C in constraints:
            cons.append({
                'type': 'ineq',
                'fun': lambda x, A=A, B=B, C=C: C - A * x[0] - B * x[1]
            })

        bounds = [
            (-self.v_max, self.v_max),
            (-self.w_max, self.w_max)
        ]

        result = minimize(
            objective,
            x0=[v_goal, w_goal],
            constraints=cons,
            bounds=bounds,
            method='SLSQP'
        )

        if result.success:
            return result.x

        return np.array([0.0, 0.0])

    # ---------------------------------------------------------
    # DETECCIÓN DE DEADLOCK
    # ---------------------------------------------------------

    def is_deadlock(self, v, w):
        return (abs(v) < self.deadlock_tol and abs(w) < self.deadlock_tol)

    # ---------------------------------------------------------
    # FUNCIÓN DE LYAPUNOV V(z)
    # ---------------------------------------------------------

    def lyapunov(self, a, alpha):
        return 0.5 * a**2 + 0.5 * alpha**2

    # ---------------------------------------------------------
    # VÉRTICES DEL FVP
    # ---------------------------------------------------------

    def get_fvp_vertices(self, constraints):
        """
        Calcula los vértices del polígono factible intersectando
        pares de restricciones lineales A*v + B*w >= C,
        más los límites de velocidad.
        """

        # Agregar límites de velocidad como restricciones
        all_constraints = list(constraints) + [
            ( 1.0,  0.0,  self.v_max),   # v <= v_max
            (-1.0,  0.0,  self.v_max),   # -v <= v_max → v >= -v_max
            ( 0.0,  1.0,  self.w_max),   # w <= w_max
            ( 0.0, -1.0,  self.w_max),   # -w <= w_max → w >= -w_max
        ]

        vertices = []
        n = len(all_constraints)

        for i in range(n):
            for j in range(i + 1, n):

                Ai, Bi, Ci = all_constraints[i]
                Aj, Bj, Cj = all_constraints[j]

                det = Ai * Bj - Aj * Bi

                if abs(det) < 1e-9:
                    continue

                v = (Ci * Bj - Cj * Bi) / det
                w = (Ai * Cj - Aj * Ci) / det

                # Verificar que el punto satisface todas las constraints
                feasible = True

                for Ak, Bk, Ck in all_constraints:
                    if Ak * v + Bk * w > Ck + 1e-6:
                        feasible = False
                        break

                if feasible:
                    vertices.append(np.array([v, w]))

        return vertices

    # ---------------------------------------------------------
    # DETERMINAR SEARCH DIRECTION Y CONSTRAINT TO FOLLOW
    # ---------------------------------------------------------

    def get_blocking_info(self, constraints):
        """
        Identifica la constraint bloqueante y la dirección de búsqueda.
        La constraint bloqueante es la que tiene C más grande (más restrictiva).
        La dirección depende del signo de B:
          B > 0 → obstáculo a la izquierda → search direction: ccw
          B < 0 → obstáculo a la derecha  → search direction: cw
        """

        if not constraints:
            return None, None

        # La constraint más activa es la que más restringe (C mayor)
        blocking = max(constraints, key=lambda c: c[2])

        A, B, C = blocking

        # B > 0 → obstáculo izquierda → rodear counterclockwise
        # B < 0 → obstáculo derecha  → rodear clockwise
        search_direction = 'ccw' if B > 0 else 'cw'

        return blocking, search_direction

    # ---------------------------------------------------------
    # VÉRTICE A APLICAR EN BOUNDARY FOLLOWING
    # ---------------------------------------------------------

    def get_boundary_velocity(self, constraints):
        """
        Calcula sL o sR — los dos extremos de la constraint bloqueante
        intersectada con los limites de velocidad.
        """

        if self.constraint_to_follow is None:
            return np.array([0.0, 0.0])

        A, B, C = self.constraint_to_follow

        candidates = []

        if abs(B) > 1e-9:
            w = (C - A * self.v_max) / B
            if -self.w_max <= w <= self.w_max:
                candidates.append(np.array([self.v_max, w]))

        if abs(B) > 1e-9:
            w = (C - A * (-self.v_max)) / B
            if -self.w_max <= w <= self.w_max:
                candidates.append(np.array([-self.v_max, w]))

        if abs(A) > 1e-9:
            v = (C - B * self.w_max) / A
            if -self.v_max <= v <= self.v_max:
                candidates.append(np.array([v, self.w_max]))

        if abs(A) > 1e-9:
            v = (C - B * (-self.w_max)) / A
            if -self.v_max <= v <= self.v_max:
                candidates.append(np.array([v, -self.w_max]))

        if not candidates:
            return np.array([0.0, 0.0])

        feasible = []
        for pt in candidates:
            ok = True
            for Ak, Bk, Ck in constraints:
                if Ak * pt[0] + Bk * pt[1] > Ck + 1e-6:
                    ok = False
                    break
            if ok:
                feasible.append(pt)

        if not feasible:
            return min(candidates, key=lambda p: np.linalg.norm(p))

        if self.search_direction == 'ccw':
            return max(feasible, key=lambda p: p[1])
        else:
            return min(feasible, key=lambda p: p[1])

    # ---------------------------------------------------------
    # COMPUTE — interfaz principal con máquina de estados
    # ---------------------------------------------------------

    def compute(self, v_goal, w_goal, constraints, a, alpha):
        """
        Interfaz principal que maneja automáticamente la transición
        entre reaching_goal y boundary_following.

        Parámetros:
          v_goal, w_goal : velocidad de referencia del consenso/control
          constraints    : lista de (A, B, C) del LIDAR
          a              : distancia al goal (para V(z))
          alpha          : error de orientación al goal (para V(z))

        Retorna:
          v_safe, w_safe : velocidades a aplicar
          mode           : 'reaching_goal' | 'boundary_following'
        """

        V_current = self.lyapunov(a, alpha)

        # -------------------------------------------------------
        # MODO: REACHING THE GOAL
        # -------------------------------------------------------

        if self.mode == 'reaching_goal':

            u = self.solve_qp(v_goal, w_goal, constraints)
            v_safe, w_safe = float(u[0]), float(u[1])

            if self.is_deadlock(v_safe, w_safe) and a > 0.1:

                # Registrar V_block y entrar en boundary following
                self.V_block = V_current
                self.constraint_to_follow, self.search_direction = \
                    self.get_blocking_info(constraints)

                self.mode = 'boundary_following'

                # Aplicar inmediatamente el vértice del FVP
                u_bf = self.get_boundary_velocity(constraints)
                return float(u_bf[0]), float(u_bf[1]), self.mode

            return v_safe, w_safe, self.mode

        # -------------------------------------------------------
        # MODO: BOUNDARY FOLLOWING
        # -------------------------------------------------------

        else:

            # Verificar si ya rodeamos el obstáculo
            if self.V_block is not None and V_current < self.V_block:

                # Obstáculo rodeado — volver a reaching the goal
                self.mode = 'reaching_goal'
                self.V_block = None
                self.constraint_to_follow = None
                self.search_direction = None

                u = self.solve_qp(v_goal, w_goal, constraints)
                return float(u[0]), float(u[1]), self.mode

            # Seguir rodeando el obstáculo
            u_bf = self.get_boundary_velocity(constraints)
            return float(u_bf[0]), float(u_bf[1]), self.mode