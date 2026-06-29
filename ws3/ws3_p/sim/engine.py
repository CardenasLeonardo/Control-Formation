import math
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from robots.unicycle import Unicycle


class LeaderAgent:
    def __init__(self, nav):
        self.nav = nav

    def __call__(self, state, neighbors_dict):
        return self.nav.step(state, neighbors_dict)


class FollowerAgent:
    def __init__(self, consensus, ctrl_law, **kwargs):
        self.consensus = consensus
        self.ctrl_law  = ctrl_law
        self.kwargs    = kwargs

    def __call__(self, state, neighbors_dict):
        result = self.consensus.step(state, neighbors_dict, **self.kwargs)
        if result is None:
            return 0.0, 0.0
        a, alpha = result
        return self.ctrl_law.compute(a, alpha)


class Simulation:
    DT = 0.1
    def __init__(self, init_positions, agents, neighbor_radius=10.0):
        self.n      = len(init_positions)
        self.robots = [Unicycle(x, y, th) for x, y, th in init_positions]
        self.agents = agents
        self.R      = neighbor_radius
        self.ids    = [f'robot{i}' for i in range(self.n)]
        self.t      = 0.0

        self.history = {
            'time':   [],
            'states': [[] for _ in range(self.n)],
            'cmds':   [[] for _ in range(self.n)],
        }

    def _neighbors(self, i):
        xi, yi, _ = self.robots[i].state()
        result = {}
        for j, robot in enumerate(self.robots):
            if j == i:
                continue
            xj, yj, thj = robot.state()
            if math.hypot(xj - xi, yj - yi) <= self.R:
                result[self.ids[j]] = (xj, yj, thj)
        return result

    def step(self):
        cmds = []
        for i, (robot, agent) in enumerate(zip(self.robots, self.agents)):
            v, w = agent(robot.state(), self._neighbors(i))
            cmds.append((v, w))

        for i, (robot, (v, w)) in enumerate(zip(self.robots, cmds)):
            robot.step(v, w, self.DT)
            self.history['states'][i].append(robot.state())
            self.history['cmds'][i].append((v, w))

        self.t += self.DT
        self.history['time'].append(self.t)

    def run(self, t_final, verbose=True):
        n_steps = int(t_final / self.DT)
        report  = max(1, n_steps // 20)

        for k in range(n_steps):
            self.step()
            if verbose and k % report == 0:
                print(f"  t = {self.t:6.1f} s / {t_final:.0f} s", end='\r')

        if verbose:
            print(f"  Simulación completada: {self.t:.1f} s           ")

        return self.history
