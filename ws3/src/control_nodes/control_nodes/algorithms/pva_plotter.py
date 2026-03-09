import numpy as np
import matplotlib.pyplot as plt


class PVAPlotter:

    def __init__(self, vmax=1.0, wmax=1.0):
        self.vmax = vmax
        self.wmax = wmax

        plt.ion()
        self.fig, self.ax = plt.subplots()

    def plot_constraints(self, constraints, goal=None, safe=None):
        self.ax.clear()

        v = np.linspace(-self.vmax, self.vmax, 400)

        # Límites del espacio de velocidades
        self.ax.axvline(self.vmax, color='black', linewidth=1.5)
        self.ax.axvline(-self.vmax, color='black', linewidth=1.5)
        self.ax.axhline(self.wmax, color='black', linewidth=1.5)
        self.ax.axhline(-self.wmax, color='black', linewidth=1.5)

        # Constraints PVA
        for A, B, C in constraints:
            if abs(B) < 1e-8:
                # recta vertical: A*v = C
                if abs(A) > 1e-8:
                    v_line = C / A
                    self.ax.axvline(v_line, color='red', linewidth=1)
                continue

            w = (C - A * v) / B
            self.ax.plot(v, w, color='red', linewidth=1)

        # Punto deseado
        if goal is not None:
            vg, wg = goal
            self.ax.plot(vg, wg, 'bo', markersize=8, label='goal')

        # Punto seguro
        if safe is not None:
            vs, ws = safe
            self.ax.plot(vs, ws, 'go', markersize=8, label='safe')

        self.ax.set_xlim(-self.vmax, self.vmax)
        self.ax.set_ylim(-self.wmax, self.wmax)

        self.ax.set_xlabel("v")
        self.ax.set_ylabel("ω")
        self.ax.set_title("Espacio de velocidades PVA")
        self.ax.grid(True)

        handles, labels = self.ax.get_legend_handles_labels()
        if labels:
            self.ax.legend()

        plt.pause(0.001)