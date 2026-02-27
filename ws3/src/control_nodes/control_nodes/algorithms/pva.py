import numpy as np


class PVA:

    def __init__(self,d_safe=0.3,d_influence=1.0,xi=1.0,v_max=0.5,w_max=1.5):

        self.d_safe = d_safe
        self.d_influence = d_influence
        self.xi = xi

        self.v_max = v_max
        self.w_max = w_max

    # ---------------------------------------------------------

    def build_constraints(self, robot_pose, obstacles):
        """
        Construye restricciones lineales Av <= c
        """

        x, y, theta = robot_pose

        A = []
        c = []

        for obs in obstacles:

            ox, oy = obs

            dx = ox - x
            dy = oy - y

            d = np.sqrt(dx**2 + dy**2)

            # Solo si está en zona de influencia
            if d > self.d_influence:
                continue

            # Vector normal hacia obstáculo
            n = np.array([dx, dy]) / d

            # Dirección longitudinal del robot
            m = np.array([np.cos(theta), np.sin(theta)])

            # Aproximamos punto P como centro del robot
            # (Después lo refinamos)
            RP = np.array([0.0, 0.0])

            # Coeficientes lineales
            a = np.dot(m, n)

            # Para robot diferencial:
            # k x RP = (-RP_y, RP_x)
            k_cross_RP = np.array([-RP[1], RP[0]])
            b = np.dot(k_cross_RP, n)

            # Límite derecho
            c_i = self.xi * (d - self.d_safe) / (self.d_influence - self.d_safe)

            A.append([a, b])
            c.append(c_i)

        return np.array(A), np.array(c)

    # ---------------------------------------------------------

    def project_velocity(self, u_goal, A, c):
        """
        Proyecta u_goal en el conjunto Av <= c
        """

        u = np.array(u_goal)

        for i in range(len(A)):
            if np.dot(A[i], u) > c[i]:

                # Proyección ortogonal sobre la recta Ai u = ci
                Ai = A[i]
                u = u - (np.dot(Ai, u) - c[i]) / np.dot(Ai, Ai) * Ai

        return u

    # ---------------------------------------------------------

    def apply_limits(self, u):

        v = np.clip(u[0], -self.v_max, self.v_max)
        w = np.clip(u[1], -self.w_max, self.w_max)

        return np.array([v, w])

    # ---------------------------------------------------------

    def filter(self, u_goal, robot_pose, obstacles):

        A, c = self.build_constraints(robot_pose, obstacles)

        if len(A) > 0:
            u = self.project_velocity(u_goal, A, c)
        else:
            u = np.array(u_goal)

        u = self.apply_limits(u)

        return u[0], u[1]