import numpy as np
import matplotlib.pyplot as plt


class PVAPlotter:

    def __init__(self, vmax=1.0, wmax=1.0):

        self.vmax = vmax
        self.wmax = wmax

        plt.ion()

        self.fig, self.ax = plt.subplots(figsize=(6,6))

        plt.show(block=False)


    # --------------------------------------------------
    # CLIP POLYGON WITH HALFPLANE
    # --------------------------------------------------

    def clip_polygon(self, polygon, A, B, C):

        new_poly = []

        for i in range(len(polygon)):

            x1, y1 = polygon[i]
            x2, y2 = polygon[(i+1) % len(polygon)]

            f1 = A*x1 + B*y1 - C
            f2 = A*x2 + B*y2 - C

            # punto 1 dentro
            if f1 >= 0:
                new_poly.append((x1, y1))

            # segmento cruza la recta
            if f1 * f2 < 0:

                t = f1 / (f1 - f2)

                xi = x1 + t*(x2-x1)
                yi = y1 + t*(y2-y1)

                new_poly.append((xi, yi))

        return new_poly


    # --------------------------------------------------
    # MAIN PLOT FUNCTION
    # --------------------------------------------------

    def plot_constraints(self, constraints, goal=None, safe=None):

        self.ax.clear()

        # --------------------------------
        # POLIGONO INICIAL (RECTANGULO)
        # --------------------------------

        polygon = [
            (-self.vmax, -self.wmax),
            ( self.vmax, -self.wmax),
            ( self.vmax,  self.wmax),
            (-self.vmax,  self.wmax)
        ]

        # --------------------------------
        # RECORTAR CONSTRAINTS
        # --------------------------------

        for A, B, C in constraints:

            polygon = self.clip_polygon(polygon, A, B, C)

            if not polygon:
                break


        # --------------------------------
        # DIBUJAR REGION FACTIBLE
        # --------------------------------

        if polygon:

            xs = [p[0] for p in polygon]
            ys = [p[1] for p in polygon]

            self.ax.fill(
                xs,
                ys,
                color='#b6ffb6',
                alpha=0.6,
                zorder=0,
                label="región factible"
            )

            # vertices
            self.ax.scatter(xs, ys, color='darkgreen', s=20)


        # --------------------------------
        # LIMITES DEL ESPACIO
        # --------------------------------

        self.ax.axvline(self.vmax, color='black', linewidth=1.5)
        self.ax.axvline(-self.vmax, color='black', linewidth=1.5)

        self.ax.axhline(self.wmax, color='black', linewidth=1.5)
        self.ax.axhline(-self.wmax, color='black', linewidth=1.5)


        # ejes centrales
        self.ax.axvline(0, color='gray', linestyle='--')
        self.ax.axhline(0, color='gray', linestyle='--')


        # --------------------------------
        # RECTAS DE LAS CONSTRAINTS
        # --------------------------------

        v = np.linspace(-self.vmax, self.vmax, 200)

        for A, B, C in constraints:

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
                vg,
                wg,
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
                vs,
                ws,
                color='green',
                s=80,
                label='safe',
                zorder=5
            )


        # --------------------------------
        # CONFIGURACION DEL PLOT
        # --------------------------------

        self.ax.set_xlim(-self.vmax, self.vmax)
        self.ax.set_ylim(-self.wmax, self.wmax)

        self.ax.set_xlabel("v (velocidad lineal)")
        self.ax.set_ylabel("ω (velocidad angular)")

        self.ax.set_title("Polígono de Velocidades Admisibles (PVA)")

        self.ax.set_aspect('equal')

        self.ax.grid(True)


        handles, labels = self.ax.get_legend_handles_labels()

        if labels:
            self.ax.legend()


        plt.pause(0.001)