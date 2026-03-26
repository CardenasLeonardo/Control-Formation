import os
import time

import rclpy
from rclpy.node import Node

from multi_robot_interfaces.msg import RobotState

import matplotlib.pyplot as plt


class StatesPlotterV2(Node):

    def __init__(self):

        super().__init__('states_plotter_v2')

        self.sub = self.create_subscription(
            RobotState,
            '/robot_states_plot',
            self.state_callback,
            10
        )

        # almacenamiento
        self.data = {}

        # líneas activas por robot { robot: [line_xy, line_x, line_y] }
        self.lines = {}

        self.start_time = time.time()

        # carpeta para guardar figuras
        self.save_dir = "figures_plotter_v2"
        os.makedirs(self.save_dir, exist_ok=True)

        # configuración de fuente tipo paper
        plt.rcParams.update({
            "font.family": "Times New Roman",
            "mathtext.fontset": "stix",
            "font.size": 11
        })

        plt.ion()

        self.fig, self.ax = plt.subplots(1, 3, figsize=(15, 5))

        # configuración fija de ejes (solo una vez)
        self.ax[0].set_title("Robot trajectory")
        self.ax[1].set_title("Evolution of x")
        self.ax[2].set_title("Evolution of y")

        self.ax[0].set_xlabel("x (m)")
        self.ax[0].set_ylabel("y (m)")

        self.ax[1].set_xlabel("Time (s)")
        self.ax[1].set_ylabel("x (m)")

        self.ax[2].set_xlabel("Time (s)")
        self.ax[2].set_ylabel("y (m)")

        for a in self.ax:
            a.grid(True)

        self.fig.tight_layout()

        plt.show(block=False)

        self.timer = self.create_timer(0.1, self.update_plot)

        self.get_logger().info("States Plotter V2 iniciado")

    # -------------------------------------------------

    def state_callback(self, msg):

        robot = msg.robot_id

        t = time.time() - self.start_time

        if robot not in self.data:

            self.data[robot] = {"t": [], "x": [], "y": []}

        self.data[robot]["t"].append(t)
        self.data[robot]["x"].append(msg.x)
        self.data[robot]["y"].append(msg.y)

    # -------------------------------------------------

    def update_plot(self):

        if not self.data:
            return

        needs_legend = False

        for robot, d in self.data.items():

            if robot not in self.lines:

                # crear líneas nuevas solo la primera vez
                l0, = self.ax[0].plot([], [], label=robot, linewidth=2)
                l1, = self.ax[1].plot([], [], label=robot, linewidth=2)
                l2, = self.ax[2].plot([], [], label=robot, linewidth=2)

                self.lines[robot] = [l0, l1, l2]

                needs_legend = True

            x = d["x"]
            y = d["y"]
            t = d["t"]

            self.lines[robot][0].set_data(x, y)
            self.lines[robot][1].set_data(t, x)
            self.lines[robot][2].set_data(t, y)

        # re-escalar ejes para acomodar nuevos datos
        for a in self.ax:
            a.relim()
            a.autoscale_view()

        self.ax[0].set_aspect('equal', adjustable='datalim')

        if needs_legend:
            for a in self.ax:
                a.legend()

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

    # -------------------------------------------------

    def save_figures(self):

        if not self.data:
            return

        # ---------- trayectoria ----------
        fig, ax = plt.subplots(figsize=(6.5, 4))

        for robot, d in self.data.items():
            ax.plot(d["x"], d["y"], label=robot, linewidth=2)

        ax.set_title("Robot trajectory")
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.grid(True)
        ax.set_aspect('equal')
        ax.legend()

        fig.savefig(f"{self.save_dir}/trajectory.pdf", bbox_inches="tight")
        plt.close(fig)

        # ---------- evolución x ----------
        fig, ax = plt.subplots(figsize=(6.5, 4))

        for robot, d in self.data.items():
            ax.plot(d["t"], d["x"], label=robot, linewidth=2)

        ax.set_title("Evolution of x")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("x (m)")
        ax.grid(True)
        ax.legend()

        fig.savefig(f"{self.save_dir}/x_evolution.pdf", bbox_inches="tight")
        plt.close(fig)

        # ---------- evolución y ----------
        fig, ax = plt.subplots(figsize=(6.5, 4))

        for robot, d in self.data.items():
            ax.plot(d["t"], d["y"], label=robot, linewidth=2)

        ax.set_title("Evolution of y")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("y (m)")
        ax.grid(True)
        ax.legend()

        fig.savefig(f"{self.save_dir}/y_evolution.pdf", bbox_inches="tight")
        plt.close(fig)

        self.get_logger().info(f"Figuras guardadas en: {self.save_dir}")


# -------------------------------------------------

def main(args=None):

    rclpy.init(args=args)

    node = StatesPlotterV2()

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