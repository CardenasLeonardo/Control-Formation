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
                 n_rays_used=None):

        self.d_safe      = d_safe
        self.d_influence = d_influence
        self.xi          = xi
        self.rp          = rp
        self.v_max       = v_max
        self.w_max       = w_max
        self.n_rays_used = n_rays_used

    # --------------------------------------------------
    # BUILD CONSTRAINTS
    # --------------------------------------------------

    def build_constraints(self, ranges):

        n = len(ranges)
        if self.n_rays_used is None or self.n_rays_used >= n:
            idxs = range(n)
        else:
            step = n // self.n_rays_used
            idxs = range(0, n, step)

        constraints = []
        for i in idxs:
            d = ranges[i]
            if d <= 0 or d > self.d_influence:
                continue

            angle = -math.pi + i * (2.0 * math.pi / n)
            A = math.cos(angle)
            B = self.rp * math.sin(angle)
            ratio = (d - self.d_safe) / (self.d_influence - self.d_safe)
            C = self.xi * ratio

            constraints.append((A, B, C))

        constraints.append(( 1.0,  0.0, self.v_max))
        constraints.append((-1.0,  0.0, self.v_max))
        constraints.append(( 0.0,  1.0, self.w_max))
        constraints.append(( 0.0, -1.0, self.w_max))

        return constraints

    # --------------------------------------------------
    # SOLVE QP
    # --------------------------------------------------

    def solve_qp(self, u_ref, constraints):
        v_ref, w_ref = u_ref
        v0 = float(np.clip(v_ref, -self.v_max, self.v_max))
        w0 = float(np.clip(w_ref, -self.w_max, self.w_max))

        if not constraints:
            return np.array([v0, w0])

        A = np.array([c[0] for c in constraints])
        B = np.array([c[1] for c in constraints])
        C = np.array([c[2] for c in constraints])

        def cost(u):
            v, w = u
            return 0.5 * ((v - v_ref)**2 + (w - w_ref)**2)

        def cost_gradient(u):
            v, w = u
            return np.array([v - v_ref, w - w_ref])

        def constraint_margin(u):
            v, w = u
            return C - A * v - B * w

        def constraint_gradient(_u):
            return np.column_stack([-A, -B])

        u_star = minimize(
            fun=cost,
            x0=np.array([v0, w0]),
            method='SLSQP',
            jac=cost_gradient,
            constraints={
                'type': 'ineq',
                'fun':  constraint_margin,
                'jac':  constraint_gradient,
            },
            options={'ftol': 1e-9, 'maxiter': 100},
        )

        return u_star.x if u_star.success else np.array([0.0, 0.0])

    # --------------------------------------------------
    # COMPUTE POLYGON (vértices del FVP)
    # --------------------------------------------------

    def compute_polygon(self, constraints):

        vertices = [
            (-self.v_max, -self.w_max),
            ( self.v_max, -self.w_max),
            ( self.v_max,  self.w_max),
            (-self.v_max,  self.w_max),
        ]

        for A, B, C in constraints:
            vertices = self._clip_vertices_by_halfplane(vertices, A, B, C)
            if not vertices:
                return np.empty((0, 2))

        if len(vertices) < 3:
            return np.empty((0, 2))

        return np.array(vertices)

    @staticmethod
    def _clip_vertices_by_halfplane(vertices, A, B, C, tol=1e-9):
        """Recorta la lista de vértices (en orden) para quedarse solo con
        la parte que cumple A*v + B*w <= C, insertando un vértice nuevo
        en cada arista que cruza la recta de la restricción."""
        clipped_vertices = []
        n = len(vertices)
        for i in range(n):
            curr_vertex = vertices[i]
            prev_vertex = vertices[i - 1]

            curr_inside = A * curr_vertex[0] + B * curr_vertex[1] <= C + tol
            prev_inside = A * prev_vertex[0] + B * prev_vertex[1] <= C + tol

            if curr_inside:
                if not prev_inside:
                    clipped_vertices.append(
                        PVA._intersection_vertex(prev_vertex, curr_vertex, A, B, C)
                    )
                clipped_vertices.append(curr_vertex)
            elif prev_inside:
                clipped_vertices.append(
                    PVA._intersection_vertex(prev_vertex, curr_vertex, A, B, C)
                )

        return clipped_vertices

    @staticmethod
    def _intersection_vertex(p1, p2, A, B, C):
        """Vértice donde el segmento p1->p2 cruza la recta A*v + B*w = C."""
        v1, w1 = p1
        v2, w2 = p2
        d1 = A * v1 + B * w1 - C
        d2 = A * v2 + B * w2 - C
        t = d1 / (d1 - d2)
        return (v1 + t * (v2 - v1), w1 + t * (w2 - w1))

    # --------------------------------------------------
    # APPLY
    # --------------------------------------------------

    def apply(self, u_ref, ranges):
        constraints = self.build_constraints(ranges)
        vertices    = self.compute_polygon(constraints)
        u_star = self.solve_qp(u_ref, constraints)
        return u_star, constraints, vertices
