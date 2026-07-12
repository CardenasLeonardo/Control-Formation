import math
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
                 n_rays_used=None,
                 deadlock_thr=0.05,
                 goal_tol=0.20):

        self.d_safe       = d_safe
        self.d_influence  = d_influence
        self.xi           = xi
        self.rp           = rp
        self.v_max        = v_max
        self.w_max        = w_max
        self.n_rays_used  = n_rays_used
        self.deadlock_thr = deadlock_thr
        self.goal_tol     = goal_tol

        # --- Módulo 2 estado ---
        self._mode        = 1    # 1 = goal reaching, 2 = boundary following
        self._search_dir  = 0    # 0 = sin definir, 1 = sR (CW), -1 = sL (CCW)
        self._a_block     = 0.0  # distancia al objetivo en el momento del deadlock

    # --------------------------------------------------
    # BUILD CONSTRAINTS
    # --------------------------------------------------

    def build_constraints(self, ranges):
        n = len(ranges)
        if self.n_rays_used is None or self.n_rays_used >= n:
            idx = np.arange(n)
        else:
            idx = np.arange(self.n_rays_used) * n // self.n_rays_used

        d      = np.asarray(ranges)[idx]
        angles = -np.pi + idx * (2.0 * np.pi / n)
        A      = np.cos(angles)
        B      = self.rp * np.sin(angles)

        mask = (d > 0) & (d <= self.d_influence)
        if not np.any(mask):
            return []

        ratio = (d[mask] - self.d_safe) / (self.d_influence - self.d_safe)
        return list(zip(A[mask], B[mask], self.xi * ratio))

    # --------------------------------------------------
    # SOLVE QP
    # --------------------------------------------------

    def solve_qp(self, v_ref, w_ref, constraints):
        v0 = float(np.clip(v_ref, -self.v_max, self.v_max))
        w0 = float(np.clip(w_ref, -self.w_max, self.w_max))

        if not constraints:
            return np.array([v0, w0])

        A = np.array([c[0] for c in constraints])
        B = np.array([c[1] for c in constraints])
        C = np.array([c[2] for c in constraints])

        result = minimize(
            fun=lambda x: 0.5 * ((x[0] - v_ref)**2 + (x[1] - w_ref)**2),
            x0=np.array([v0, w0]),
            method='SLSQP',
            jac=lambda x: np.array([x[0] - v_ref, x[1] - w_ref]),
            constraints={
                'type': 'ineq',
                'fun':  lambda x: C - A * x[0] - B * x[1],
                'jac':  lambda _: np.column_stack([-A, -B]),
            },
            bounds=[(-self.v_max, self.v_max), (-self.w_max, self.w_max)],
            options={'ftol': 1e-9, 'maxiter': 100},
        )

        return result.x if result.success else np.array([0.0, 0.0])

    # --------------------------------------------------
    # COMPUTE POLYGON (vértices del FVP)
    # --------------------------------------------------

    def compute_polygon(self, constraints):
        A_box = np.array([ 1.0, -1.0,  0.0,  0.0])
        B_box = np.array([ 0.0,  0.0,  1.0, -1.0])
        C_box = np.array([self.v_max, self.v_max, self.w_max, self.w_max])

        if constraints:
            A_all = np.concatenate([[c[0] for c in constraints], A_box])
            B_all = np.concatenate([[c[1] for c in constraints], B_box])
            C_all = np.concatenate([[c[2] for c in constraints], C_box])
        else:
            A_all, B_all, C_all = A_box, B_box, C_box

        points = []
        n = len(A_all)
        for i in range(n):
            for j in range(i + 1, n):
                det = A_all[i] * B_all[j] - A_all[j] * B_all[i]
                if abs(det) < 1e-9:
                    continue
                v = (C_all[i] * B_all[j] - C_all[j] * B_all[i]) / det
                w = (A_all[i] * C_all[j] - A_all[j] * C_all[i]) / det
                if np.all(A_all * v + B_all * w <= C_all + 1e-7):
                    points.append([v, w])

        if len(points) < 3:
            return np.empty((0, 2))

        pts = np.unique(np.round(np.array(points, dtype=float), 8), axis=0)
        if len(pts) < 3:
            return np.empty((0, 2))

        center = np.mean(pts, axis=0)
        order  = np.argsort(np.arctan2(pts[:, 1] - center[1], pts[:, 0] - center[0]))
        return pts[order]

    # --------------------------------------------------
    # MÓDULO 2 — search direction
    # --------------------------------------------------

    def _compute_search_dir(self, constraints):
        if not constraints:
            return 1
        # La restricción más bloqueante es la de menor C
        idx = int(np.argmin([c[2] for c in constraints]))
        b = constraints[idx][1]   # B = rp * sin(ángulo del rayo)
        # B > 0 → rayo desde la izquierda → obstáculo izquierda → girar derecha (sR)
        # B < 0 → rayo desde la derecha  → obstáculo derecha   → girar izquierda (sL)
        return 1 if b >= 0 else -1

    # --------------------------------------------------
    # MÓDULO 2 — selección de vértice
    # --------------------------------------------------

    def _boundary_vertex(self, vertices):
        if len(vertices) == 0:
            return np.array([0.0, 0.0])
        if self._search_dir == 1:          # sR → mínima w (giro horario)
            return vertices[np.argmin(vertices[:, 1])]
        else:                              # sL → máxima w (giro antihorario)
            return vertices[np.argmax(vertices[:, 1])]

    # --------------------------------------------------
    # APPLY
    # --------------------------------------------------

    def apply(self, v_ref, w_ref, ranges, a=None):
        constraints = self.build_constraints(ranges) if ranges else []
        vertices    = self.compute_polygon(constraints)

        if a is None:
            u = self.solve_qp(v_ref, w_ref, constraints)
            return float(u[0]), float(u[1]), constraints, vertices, 1, 0

        # --- Módulo 1 ---
        if self._mode == 1:
            u = self.solve_qp(v_ref, w_ref, constraints)
            if np.linalg.norm(u) < self.deadlock_thr and a > self.goal_tol:
                self._mode       = 2
                self._a_block    = a
                self._search_dir = self._compute_search_dir(constraints)
            else:
                return float(u[0]), float(u[1]), constraints, vertices, 1, 0

        # --- Módulo 2 ---
        if a < self._a_block:
            self._mode       = 1
            self._search_dir = 0
            u = self.solve_qp(v_ref, w_ref, constraints)
            return float(u[0]), float(u[1]), constraints, vertices, 1, 0

        u = self._boundary_vertex(vertices)
        return float(u[0]), float(u[1]), constraints, vertices, 2, self._search_dir
