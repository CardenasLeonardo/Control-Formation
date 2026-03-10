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
                 n_rays=360):

        self.d_safe = d_safe
        self.d_influence = d_influence
        self.xi = xi

        self.v_max = v_max
        self.w_max = w_max

        # Precalcular A y B
        angles = angle_min + np.arange(n_rays)*angle_increment

        self.A = -np.cos(angles)
        self.B = -rp*np.sin(angles)

    # ---------------------------------------------------------

    def build_constraints(self, ranges):

        d = np.asarray(ranges)

        mask = (d > 0) & (d <= self.d_influence)

        if not np.any(mask):
            return []

        d = d[mask]

        ratio = (d - self.d_safe)/(self.d_influence - self.d_safe)

        C = -self.xi*ratio

        A = self.A[mask]
        B = self.B[mask]

        constraints = list(zip(A,B,C))

        return constraints

    # ---------------------------------------------------------

    def solve_qp(self, v_goal, w_goal, constraints):

        # función objetivo
        def objective(x):

            v, w = x

            return 0.5*((v - v_goal)**2 + (w - w_goal)**2)

        # constraints PVA
        cons = []

        for A,B,C in constraints:

            cons.append({
                'type': 'ineq',
                'fun': lambda x, A=A, B=B, C=C:
                    A*x[0] + B*x[1] - C
            })

        # límites de velocidad
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

        # fallback
        return np.array([0.0,0.0])