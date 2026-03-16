import os
import math
import rclpy
from rclpy.node import Node

import numpy as np
import matplotlib.pyplot as plt

from multi_robot_interfaces.msg import PVAConstraints


class PVAPlotter(Node):

    def __init__(self):

        super().__init__('pva_plotter')

        # ---------------------------------------------
        # Parámetros de límites de velocidad
        # ---------------------------------------------
        self.declare_parameter('vmax', 1.0)
        self.declare_parameter('wmax', 1.0)

        self.vmax = float(self.get_parameter('vmax').value)
        self.wmax = float(self.get_parameter('wmax').value)

        # ---------------------------------------------
        # Subscriber
        # ---------------------------------------------
        self.sub = self.create_subscription(
            PVAConstraints,
            '/pva_constraints',
            self.pva_callback,
            10
        )

        # ---------------------------------------------
        # Datos por robot
        # ---------------------------------------------
        self.data = {}

        # ---------------------------------------------
        # Ejes de visualización
        # ---------------------------------------------
        self.fig_main = None
        self.axes = {}

        # ---------------------------------------------
        # Carpeta de salida
        # ---------------------------------------------
        self.save_dir = "figures_pva"
        os.makedirs(self.save_dir, exist_ok=True)

        # ---------------------------------------------
        # Estilo tipo paper
        # ---------------------------------------------
        plt.rcParams.update({
            "font.family": "Times New Roman",
            "mathtext.fontset": "stix",
            "font.size": 11
        })

        plt.ion()

        self.timer = self.create_timer(0.2, self.update_plot)

        self.get_logger().info("PVA Plotter iniciado")

    # --------------------------------------------------
    # CALLBACK
    # --------------------------------------------------

    def pva_callback(self, msg):

        robot = msg.robot_id

        self.data[robot] = {
            "a": np.array(msg.a, dtype=float),
            "b": np.array(msg.b, dtype=float),
            "c": np.array(msg.c, dtype=float),
            "v_goal": float(msg.v_goal),
            "w_goal": float(msg.w_goal),
            "v_star": float(msg.v_star),
            "w_star": float(msg.w_star)
        }

    # --------------------------------------------------
    # CONSTRUIR LAYOUT DINÁMICO
    # --------------------------------------------------

    def build_layout(self):

        n = len(self.data)

        cols = math.ceil(math.sqrt(n))
        rows = math.ceil(n / cols)

        self.fig_main, axs = plt.subplots(rows, cols, figsize=(5*cols,4*rows))

        axs = np.atleast_1d(axs).flatten()

        self.axes = {}

        for ax, robot in zip(axs, self.data.keys()):
            self.axes[robot] = ax

        for ax in axs[len(self.data):]:
            ax.axis("off")

        plt.show(block=False)

    # --------------------------------------------------
    # AGREGAR LÍMITES DEL ROBOT COMO RESTRICCIONES
    # --------------------------------------------------

    def add_velocity_bounds(self, A, B, C):

        A_box = np.array([
            1.0,
           -1.0,
            0.0,
            0.0
        ])

        B_box = np.array([
            0.0,
            0.0,
            1.0,
           -1.0
        ])

        C_box = np.array([
            -self.vmax,
            -self.vmax,
            -self.wmax,
            -self.wmax
        ])

        A_all = np.concatenate([A, A_box])
        B_all = np.concatenate([B, B_box])
        C_all = np.concatenate([C, C_box])

        return A_all, B_all, C_all

    # --------------------------------------------------
    # CALCULAR POLÍGONO FACTIBLE
    # --------------------------------------------------

    def compute_polygon(self, A, B, C):

        points = []
        n = len(A)

        for i in range(n):
            for j in range(i + 1, n):

                det = A[i] * B[j] - A[j] * B[i]

                if abs(det) < 1e-9:
                    continue

                v = (C[i] * B[j] - C[j] * B[i]) / det
                w = (A[i] * C[j] - A[j] * C[i]) / det

                if np.all(A * v + B * w >= C - 1e-7):
                    points.append([v, w])

        if len(points) == 0:
            return None

        points = np.array(points, dtype=float)

        points = np.unique(np.round(points, decimals=8), axis=0)

        if len(points) < 3:
            return None

        center = np.mean(points, axis=0)
        angles = np.arctan2(points[:, 1] - center[1], points[:, 0] - center[0])
        order = np.argsort(angles)

        return points[order]

    # --------------------------------------------------
    # DIBUJAR UN ROBOT
    # --------------------------------------------------

    def draw_robot_pva(self, ax, robot, d):

        ax.cla()

        ax.set_title(f"PVA - {robot}")
        ax.set_xlabel(r"Linear velocity $v$ (m/s)")
        ax.set_ylabel(r"Angular velocity $\omega$ (rad/s)")
        ax.grid(True)

        ax.axhline(0.0, linestyle="--", color="gray", linewidth=1.0)
        ax.axvline(0.0, linestyle="--", color="gray", linewidth=1.0)

        A = d["a"]
        B = d["b"]
        C = d["c"]

        A_all, B_all, C_all = self.add_velocity_bounds(A, B, C)

        polygon = self.compute_polygon(A_all, B_all, C_all)

        if polygon is not None:

            ax.fill(
                polygon[:,0],
                polygon[:,1],
                color="lightgreen",
                alpha=0.35,
                label="Admissible region"
            )

            ax.plot(
                np.append(polygon[:,0], polygon[0,0]),
                np.append(polygon[:,1], polygon[0,1]),
                color="black",
                linewidth=1.6
            )

        ax.plot(
            d["v_goal"],
            d["w_goal"],
            marker='o',
            linestyle='None',
            markersize=8,
            color='tab:blue',
            label=r"$u_{ref}$"
        )

        ax.plot(
            d["v_star"],
            d["w_star"],
            marker='o',
            linestyle='None',
            markersize=8,
            color='tab:red',
            label=r"$u^{*}$"
        )

        margin_v = 0.15 * self.vmax
        margin_w = 0.15 * self.wmax

        ax.set_xlim(-self.vmax - margin_v, self.vmax + margin_v)
        ax.set_ylim(-self.wmax - margin_w, self.wmax + margin_w)

        ax.legend()

    # --------------------------------------------------
    # UPDATE PLOT
    # --------------------------------------------------

    def update_plot(self):

        if not self.data:
            return

        if self.fig_main is None or len(self.axes) != len(self.data):
            self.build_layout()

        for robot, d in self.data.items():

            ax = self.axes[robot]

            self.draw_robot_pva(ax, robot, d)

        self.fig_main.tight_layout()

        self.fig_main.canvas.draw_idle()
        self.fig_main.canvas.flush_events()

    # --------------------------------------------------
    # SAVE FIGURES
    # --------------------------------------------------

    def save_figures(self):

        for robot, d in self.data.items():

            fig, ax = plt.subplots(figsize=(6.5, 5.0))

            self.draw_robot_pva(ax, robot, d)

            fig.savefig(
                os.path.join(self.save_dir, f"pva_{robot}.pdf"),
                bbox_inches="tight"
            )

            plt.close(fig)

        self.get_logger().info(f"Figuras guardadas en: {self.save_dir}")


# --------------------------------------------------
# MAIN
# --------------------------------------------------

def main(args=None):

    rclpy.init(args=args)

    node = PVAPlotter()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("Guardando figuras...")
        node.save_figures()

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()