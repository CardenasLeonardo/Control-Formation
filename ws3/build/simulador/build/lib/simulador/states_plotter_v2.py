import os
import time
import threading
import subprocess

import rclpy
from rclpy.node import Node

from multi_robot_interfaces.msg import RobotState

import matplotlib
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from matplotlib.backends.backend_agg import FigureCanvasAgg
import numpy as np
from scipy.signal import savgol_filter


COLORS = plt.cm.tab10.colors


class StatesPlotterV2(Node):

    def __init__(self):

        super().__init__('states_plotter_v2')

        self.sub = self.create_subscription(
            RobotState,
            '/robot_states_plot',
            self.state_callback,
            10
        )

        self.data = {}
        self.initial_positions = {}
        self.lines = {}
        self.start_time = time.time()

        self.declare_parameter('save_dir',          '')
        self.declare_parameter('t_initial',         3.0)
        self.declare_parameter('t_final',           30.0)
        self.declare_parameter('n_robots_expected', 0)

        param_dir = self.get_parameter('save_dir').value
        self.save_dir          = param_dir or os.environ.get('EXP_SAVE_DIR', '') or 'figures_plotter_v2'
        self.t_initial         = float(self.get_parameter('t_initial').value)
        self.t_final           = float(self.get_parameter('t_final').value)
        self.n_robots_expected = int(self.get_parameter('n_robots_expected').value)

        os.makedirs(self.save_dir, exist_ok=True)

        self.first_msg_time = None
        self.initial_saved  = False
        self.final_saved    = False

        plt.rcParams.update({
            "font.family":      "Times New Roman",
            "mathtext.fontset": "stix",
            "font.size":        11
        })

        plt.ion()

        self.fig, self.ax = plt.subplots(1, 3, figsize=(15, 5))

        self.ax[0].set_title("Trayectoria del robot")
        self.ax[1].set_title("Evolución de x")
        self.ax[2].set_title("Evolución de y")

        self.ax[0].set_xlabel("x (m)")
        self.ax[0].set_ylabel("y (m)")
        self.ax[1].set_xlabel("Tiempo (s)")
        self.ax[1].set_ylabel("x (m)")
        self.ax[2].set_xlabel("Tiempo (s)")
        self.ax[2].set_ylabel("y (m)")

        for a in self.ax:
            a.grid(True)

        self.fig.tight_layout()
        plt.show(block=False)

        self.timer = self.create_timer(0.1, self.update_plot)
        self.get_logger().info(
            f"States Plotter V2 — inicial en t={self.t_initial}s, "
            f"final+shutdown en t={self.t_final}s"
        )

    # -------------------------------------------------

    def state_callback(self, msg):

        robot = msg.robot_id
        t = time.time() - self.start_time

        if robot not in self.data:
            self.data[robot] = {"t": [], "x": [], "y": [], "theta": []}

        if robot not in self.initial_positions:
            self.initial_positions[robot] = (msg.x, msg.y)

        if self.first_msg_time is None:
            self.first_msg_time = time.time()

        self.data[robot]["t"].append(t)
        self.data[robot]["x"].append(msg.x)
        self.data[robot]["y"].append(msg.y)
        self.data[robot]["theta"].append(msg.theta)

    # -------------------------------------------------

    def update_plot(self):

        if not self.data:
            return

        needs_legend = False

        for robot, d in self.data.items():

            if robot not in self.lines:
                i  = len(self.lines)
                c  = COLORS[i % len(COLORS)]
                l0, = self.ax[0].plot([], [], color=c, label=robot, linewidth=2)
                l1, = self.ax[1].plot([], [], color=c, label=robot, linewidth=2)
                l2, = self.ax[2].plot([], [], color=c, label=robot, linewidth=2)
                self.lines[robot] = [l0, l1, l2]
                needs_legend = True

            self.lines[robot][0].set_data(d["x"], d["y"])
            self.lines[robot][1].set_data(d["t"], d["x"])
            self.lines[robot][2].set_data(d["t"], d["y"])

        for a in self.ax:
            a.relim()
            a.autoscale_view()

        self.ax[0].set_aspect('equal', adjustable='datalim')

        if needs_legend:
            for a in self.ax:
                a.legend()

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

        if self.first_msg_time is not None:
            elapsed = time.time() - self.first_msg_time
            if not self.initial_saved:
                all_seen = (self.n_robots_expected > 0 and
                            len(self.initial_positions) >= self.n_robots_expected)
                time_ok  = (self.n_robots_expected <= 0 and elapsed >= self.t_initial)
                if all_seen or time_ok:
                    self.initial_saved = True
                    self.save_initial()
            if not self.final_saved and elapsed >= self.t_final:
                self.final_saved = True
                self.save_final()
                threading.Timer(2.0, self._shutdown_all).start()

    # -------------------------------------------------

    def save_initial(self):

        fig = Figure(figsize=(6, 6))
        FigureCanvasAgg(fig)
        ax = fig.add_subplot(1, 1, 1)

        for i, (robot, (x, y)) in enumerate(self.initial_positions.items()):
            c = COLORS[i % len(COLORS)]
            ax.scatter(x, y, s=140, color=c, zorder=5, label=robot)
            ax.annotate(robot, (x, y),
                        textcoords="offset points", xytext=(8, 4), fontsize=10)

        ax.set_title("Posiciones iniciales")
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.grid(True)
        ax.set_aspect('equal', adjustable='datalim')
        ax.legend()
        fig.tight_layout()

        path = os.path.join(self.save_dir, "posiciones_iniciales.pdf")
        fig.savefig(path, bbox_inches="tight")

        self.get_logger().info(f"Posiciones iniciales guardadas: {path}")

    # -------------------------------------------------

    def _draw_triangles(self, ax, x, y, theta, color, d_step=0.4, size=0.25):
        """Triángulos sólidos cada d_step metros a lo largo de la trayectoria."""
        if len(x) < 2:
            return
        cumul = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(x), np.diff(y)))])
        total = cumul[-1]
        if total < d_step:
            return
        for d_target in np.arange(d_step / 2, total, d_step):
            idx = int(np.clip(np.searchsorted(cumul, d_target, side='right') - 1,
                              0, len(x) - 1))
            xi, yi, ti = x[idx], y[idx], theta[idx]
            pts = np.array([
                [ size,        0.0        ],
                [-size * 0.5,  size * 0.45],
                [-size * 0.5, -size * 0.45],
            ])
            c_t, s_t = np.cos(ti), np.sin(ti)
            R = np.array([[c_t, -s_t], [s_t, c_t]])
            rotated = (R @ pts.T).T + np.array([xi, yi])
            ax.fill(rotated[:, 0], rotated[:, 1], color=color, zorder=3, alpha=0.85)

    def save_final(self):

        if not self.data:
            return

        fig = Figure(figsize=(15, 5))
        FigureCanvasAgg(fig)
        axes = fig.subplots(1, 3)

        for i, (robot, d) in enumerate(self.data.items()):
            c = COLORS[i % len(COLORS)]

            sort_idx = np.argsort(d["t"])
            x     = np.array(d["x"],     dtype=float)[sort_idx]
            y     = np.array(d["y"],     dtype=float)[sort_idx]
            t     = np.array(d["t"],     dtype=float)[sort_idx]
            theta = np.array(d["theta"], dtype=float)[sort_idx]

            # Suavizado Savitzky-Golay para trayectorias fluidas
            n = len(x)
            win = min(51, max(5, (n // 15) | 1))  # ventana impar, máx 51
            if n > win:
                xs = savgol_filter(x, win, 3)
                ys = savgol_filter(y, win, 3)
            else:
                xs, ys = x, y

            # Trayectoria suavizada
            axes[0].plot(xs, ys, color=c, label=robot, linewidth=1.8, zorder=2)

            # Marca posición inicial — círculo hueco
            axes[0].scatter(x[0], y[0], s=90, facecolors='white',
                            edgecolors=c, linewidths=2, zorder=4)

            # Marca posición final — estrella
            axes[0].scatter(x[-1], y[-1], color=c, marker='*',
                            s=220, zorder=5, edgecolors='black', linewidths=0.5)

            # Triángulos cada 0.4 m sobre la trayectoria suavizada
            self._draw_triangles(axes[0], xs, ys, theta, c)

            axes[1].plot(t, xs, color=c, label=robot, linewidth=1.8)
            axes[2].plot(t, ys, color=c, label=robot, linewidth=1.8)

        axes[0].set_title("Trayectoria de los robots")
        axes[0].set_xlabel("x (m)")
        axes[0].set_ylabel("y (m)")
        axes[0].grid(True)
        axes[0].set_aspect('equal')
        axes[0].legend(fontsize=8)

        axes[1].set_title("Evolución de x")
        axes[1].set_xlabel("Tiempo (s)")
        axes[1].set_ylabel("x (m)")
        axes[1].grid(True)
        axes[1].legend(fontsize=8)

        axes[2].set_title("Evolución de y")
        axes[2].set_xlabel("Tiempo (s)")
        axes[2].set_ylabel("y (m)")
        axes[2].grid(True)
        axes[2].legend(fontsize=8)

        fig.tight_layout()

        path = os.path.join(self.save_dir, "posiciones_finales.pdf")
        fig.savefig(path, bbox_inches="tight")

        self.get_logger().info(f"Posiciones finales guardadas: {path}")

    # -------------------------------------------------

    def _shutdown_all(self):

        self.get_logger().info("Cerrando simulación automáticamente...")
        subprocess.Popen(
            'pkill -9 -f gzserver; pkill -9 -f gzclient; '
            'pkill -9 -f "ros2 launch"; pkill -9 -f consenso_prom_err; '
            'pkill -9 -f navigate_individual; pkill -9 -f aire; '
            'pkill -9 -f pva_plotter; pkill -9 -f states_plotter',
            shell=True
        )

    # -------------------------------------------------

    def save_figures(self):
        if not self.final_saved:
            self.final_saved = True
            self.save_final()


# -------------------------------------------------

def main(args=None):

    rclpy.init(args=args)

    node = StatesPlotterV2()

    def esperar_enter():
        while True:
            input()
            node.get_logger().info("Guardando figuras manualmente...")
            node.save_final()

    hilo = threading.Thread(target=esperar_enter, daemon=True)
    hilo.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if not node.final_saved:
            node.save_final()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
