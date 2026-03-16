import os
import time

import rclpy
from rclpy.node import Node

from multi_robot_interfaces.msg import RobotState

import matplotlib.pyplot as plt
import matplotlib.font_manager as fm


class StatesPlotterV2(Node):

    def __init__(self):

        super().__init__('states_plotter_v2')

        self.sub = self.create_subscription(
            RobotState,
            '/robot_states_rx',
            self.state_callback,
            10
        )

        # almacenamiento
        self.data = {}

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

        plt.show(block=False)

        # timer para actualizar gráfica en vivo
        self.timer = self.create_timer(0.2, self.update_plot)

        self.get_logger().info("States Plotter V2 iniciado")

    # -------------------------------------------------

    def state_callback(self, msg):

        robot = msg.robot_id
        x = msg.x
        y = msg.y

        t = time.time() - self.start_time

        if robot not in self.data:

            self.data[robot] = {
                "t": [],
                "x": [],
                "y": []
            }

        self.data[robot]["t"].append(t)
        self.data[robot]["x"].append(x)
        self.data[robot]["y"].append(y)

    # -------------------------------------------------

    def update_plot(self):

        for a in self.ax:
            a.cla()

        # títulos
        self.ax[0].set_title("Robot trajectory")
        self.ax[1].set_title("Evolution of x")
        self.ax[2].set_title("Evolution of y")

        # etiquetas
        self.ax[0].set_xlabel("x (m)")
        self.ax[0].set_ylabel("y (m)")

        self.ax[1].set_xlabel("Time (s)")
        self.ax[1].set_ylabel("x (m)")

        self.ax[2].set_xlabel("Time (s)")
        self.ax[2].set_ylabel("y (m)")

        # grid
        for a in self.ax:
            a.grid(True)

        # graficar datos
        for robot, d in self.data.items():

            self.ax[0].plot(d["x"], d["y"], label=robot, linewidth=2)
            self.ax[1].plot(d["t"], d["x"], label=robot, linewidth=2)
            self.ax[2].plot(d["t"], d["y"], label=robot, linewidth=2)

        # escala igual para XY
        self.ax[0].set_aspect('equal', adjustable='box')

        if self.data:
            for a in self.ax:
                a.legend()

        self.fig.tight_layout()

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

    # -------------------------------------------------

    def save_figures(self):

        if not self.data:
            return

        # ---------- trayectoria ----------
        fig, ax = plt.subplots(figsize=(6.5,4))

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
        fig, ax = plt.subplots(figsize=(6.5,4))

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
        fig, ax = plt.subplots(figsize=(6.5,4))

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