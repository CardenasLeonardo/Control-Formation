import math


class ConsensusLeader:

    def __init__(self, k_leader=0.6):

        self.k_leader = k_leader


    def compute(self, state, neighbors, leader_state=None):

        xi, yi, theta = state

        # -----------------------------
        # CONSENSO ENTRE VECINOS
        # -----------------------------

        ex = 0.0
        ey = 0.0

        for xj, yj in neighbors:

            ex += (xj - xi)
            ey += (yj - yi)

        # -----------------------------
        # INFLUENCIA DEL LÍDER
        # -----------------------------

        if leader_state is not None:

            xL, yL = leader_state

            ex += self.k_leader * (xL - xi)
            ey += self.k_leader * (yL - yi)

        # -----------------------------
        # CONTROL CINEMÁTICO
        # -----------------------------

        dist = math.sqrt(ex**2 + ey**2)

        desired_theta = math.atan2(ey, ex)

        theta_error = desired_theta - theta
        theta_error = math.atan2(math.sin(theta_error), math.cos(theta_error))

        v = dist
        w = theta_error

        return v, w