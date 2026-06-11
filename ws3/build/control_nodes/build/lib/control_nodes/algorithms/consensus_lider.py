import math
from math import atan2, sqrt


class ConsensusLeader:
    """
    Protocolo de consenso con seguimiento de líder.

    Retorna el error polar (a, alpha) — magnitud y ángulo del error de consenso
    respecto a la orientación del robot.
    """

    def step(self, state, neighbors_dict, robot_id, leader_id, k_leader):
        if robot_id == leader_id:
            return None
        neighbors    = [(x, y) for rid, (x, y, _) in neighbors_dict.items() if rid != leader_id]
        leader_state = None
        if leader_id in neighbors_dict:
            xL, yL, _ = neighbors_dict[leader_id]
            leader_state = (xL, yL)
        if not neighbors and leader_state is None:
            return None
        return self.compute(state, neighbors, leader_state, k_leader)

    def compute(self, state, neighbors, leader_state=None, k_leader=2.0):

        xi, yi, theta = state

        ex = 0.0
        ey = 0.0
        n  = 0.0

        for xj, yj in neighbors:
            ex += (xj - xi)
            ey += (yj - yi)
            n  += 1.0

        if n > 0:
            ex /= n
            ey /= n

        has_leader = leader_state is not None
        if has_leader:
            xL, yL = leader_state
            # Escala con n para que el líder siempre pese k_leader veces más
            # que el grupo completo, sin importar cuántos vecinos haya
            scale = k_leader * max(1.0, n)
            ex += scale * (xL - xi)
            ey += scale * (yL - yi)

        if n == 0.0 and not has_leader:
            return 0.0, 0.0

        a = sqrt(ex**2 + ey**2)

        if a < 1e-6:
            return 0.0, 0.0

        alpha = atan2(ey, ex) - theta
        while alpha >  math.pi: alpha -= 2 * math.pi
        while alpha < -math.pi: alpha += 2 * math.pi

        return a, alpha
