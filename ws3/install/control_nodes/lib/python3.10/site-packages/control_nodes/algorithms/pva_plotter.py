import numpy as np
import matplotlib.pyplot as plt


class PVAPlotter:

    def __init__(self, vmax=1.0, wmax=1.0):

        self.vmax = vmax
        self.wmax = wmax

        plt.ion()

        self.fig, self.ax = plt.subplots(figsize=(6,6))


    def plot_constraints(self, constraints, goal=None, safe=None):

        self.ax.clear()

        # --------------------------------
        # MALLA DEL ESPACIO DE VELOCIDADES
        # --------------------------------

        v = np.linspace(-self.vmax, self.vmax, 200)
        w = np.linspace(-self.wmax, self.wmax, 200)

        V, W = np.meshgrid(v, w)

        feasible = np.ones_like(V, dtype=bool)

        # evaluar constraints
        for A, B, C in constraints:

            feasible &= (A*V + B*W >= C)

        # --------------------------------
        # DIBUJAR REGION FACTIBLE
        # --------------------------------

        self.ax.contourf(
            V,
            W,
            feasible,
            levels=[0.5,1],
            colors=['#d0ffd0'],
            alpha=0.5
        )

        # --------------------------------
        # LIMITES DEL ESPACIO
        # --------------------------------

        self.ax.axvline(self.vmax, color='black', linewidth=1.5)
        self.ax.axvline(-self.vmax, color='black', linewidth=1.5)
        self.ax.axhline(self.wmax, color='black', linewidth=1.5)
        self.ax.axhline(-self.wmax, color='black', linewidth=1.5)

        # ejes
        self.ax.axvline(0, color='gray', linestyle='--')
        self.ax.axhline(0, color='gray', linestyle='--')

        # --------------------------------
        # RECTAS DE LAS CONSTRAINTS
        # --------------------------------

        for A,B,C in constraints:

            if abs(B) < 1e-8:

                if abs(A) > 1e-8:

                    v_line = C/A

                    self.ax.axvline(v_line, color='red')

                continue

            w_line = (C - A*v)/B

            self.ax.plot(v, w_line, color='red')

        # --------------------------------
        # GOAL
        # --------------------------------

        if goal is not None:

            vg, wg = goal

            self.ax.scatter(
                vg, wg,
                color='blue',
                s=80,
                label='goal',
                zorder=5
            )

        # --------------------------------
        # SAFE
        # --------------------------------

        if safe is not None:

            vs, ws = safe

            self.ax.scatter(
                vs, ws,
                color='green',
                s=80,
                label='safe',
                zorder=5
            )

        # --------------------------------

        self.ax.set_xlim(-self.vmax, self.vmax)
        self.ax.set_ylim(-self.wmax, self.wmax)

        self.ax.set_xlabel("v")
        self.ax.set_ylabel("ω")

        self.ax.set_title("Espacio de velocidades PVA")

        self.ax.set_aspect('equal')

        self.ax.grid(True)

        handles, labels = self.ax.get_legend_handles_labels()

        if labels:
            self.ax.legend()

        plt.pause(0.001)