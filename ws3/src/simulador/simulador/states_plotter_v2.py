import os
import math
import time
import threading
import subprocess

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from nav_msgs.msg import Odometry
from multi_robot_interfaces.msg import RobotState

from matplotlib.figure import Figure
from matplotlib.backends.backend_agg import FigureCanvasAgg
import matplotlib.pyplot as plt
import numpy as np

plt.rcParams.update({
    "font.family":      "serif",
    "font.serif":       ["Times New Roman"],
    "mathtext.fontset": "stix",
    "font.size":        11,
})

COLORS = plt.cm.tab10.colors


def _compute_offset(follower_index, n_robots, theta_v, angle_v, d):
    """Offset (rx, ry) del robot `follower_index` dentro de la V, para el
    heading `theta_v` de la estructura virtual. Misma fórmula que
    vs_node._compute_offsets / consensus_formation._compute_offsets."""
    if follower_index == 0:
        return (0.0, 0.0)

    n_followers = n_robots - 1
    n_left      = n_followers // 2

    def rot(theta, vx, vy):
        c, s = math.cos(theta), math.sin(theta)
        return c * vx - s * vy, s * vx + c * vy

    if follower_index <= n_left:
        return rot(theta_v + angle_v, -follower_index * d, 0.0)
    else:
        k = follower_index - n_left
        return rot(theta_v - angle_v, -k * d, 0.0)


class StatesPlotterV2(Node):

    def __init__(self):

        super().__init__('states_plotter_v2')

        self.data = {}
        self.initial_positions = {}
        self.start_time = time.time()

        self.declare_parameter('save_dir',          '')
        self.declare_parameter('t_initial',         3.0)
        self.declare_parameter('t_final',           30.0)
        self.declare_parameter('n_robots_expected', 0)
        self.declare_parameter('trajectory_only',   False)
        # Trayectoria + error euclidiano respecto al offset individual dentro
        # de la V (requiere /virtual_structure, publicado por vs_node)
        self.declare_parameter('error_vs_virtual_structure', False)
        # Parámetros de snapshots de formación
        self.declare_parameter('formation_snapshots',  False)
        self.declare_parameter('snap_interval_s',      15.0)
        self.declare_parameter('formation_leader_id',  'robot0')
        self.declare_parameter('formation_n_followers', 4)
        self.declare_parameter('formation_angle_v',    0.7854)
        self.declare_parameter('formation_d',          1.0)

        param_dir = self.get_parameter('save_dir').value
        self.save_dir              = param_dir or os.environ.get('EXP_SAVE_DIR', '') or 'figures_plotter_v2'
        self.t_initial             = float(self.get_parameter('t_initial').value)
        self.t_final               = float(self.get_parameter('t_final').value)
        self.n_robots_expected     = int(self.get_parameter('n_robots_expected').value)
        self.trajectory_only       = bool(self.get_parameter('trajectory_only').value)
        self.error_vs_virtual_structure = bool(self.get_parameter('error_vs_virtual_structure').value)
        self.formation_snapshots   = bool(self.get_parameter('formation_snapshots').value)
        self.snap_interval_s       = float(self.get_parameter('snap_interval_s').value)
        self.formation_leader_id   = str(self.get_parameter('formation_leader_id').value)
        self.formation_n_followers = int(self.get_parameter('formation_n_followers').value)
        self.formation_angle_v     = float(self.get_parameter('formation_angle_v').value)
        self.formation_d           = float(self.get_parameter('formation_d').value)

        os.makedirs(self.save_dir, exist_ok=True)

        self.first_msg_time = None
        self.initial_saved  = False
        self.final_saved    = False
        self._lock          = threading.Lock()

        # Snapshots de formación
        self.snapshots        = []   # lista de {t, robots: {rid: (x,y,theta)}}
        self.last_snap_t      = None

        # Serie temporal de la estructura virtual (para el error vs offset ideal)
        self.vs_data = {'t': [], 'x': [], 'y': [], 'theta': []}
        if self.error_vs_virtual_structure:
            self.create_subscription(RobotState, '/virtual_structure', self._vs_cb, 100)

        # Suscripción directa a odom de cada robot (tasa máxima de Gazebo)
        if self.n_robots_expected > 0:
            for i in range(self.n_robots_expected):
                rid = f'robot{i}'
                self.create_subscription(
                    Odometry,
                    f'/{rid}/odom',
                    lambda msg, r=rid: self._odom_cb(msg, r),
                    100
                )
        else:
            self.create_subscription(
                RobotState,
                '/robot_states_plot',
                self.state_callback,
                100
            )

        # Timer solo para vigilar el tiempo — sin matplotlib
        self.timer = self.create_timer(0.5, self._check_timing)

        self.get_logger().info(
            f"States Plotter V2 — inicial en t={self.t_initial}s, "
            f"final+shutdown en t={self.t_final}s"
        )

    # -------------------------------------------------

    def _odom_cb(self, msg, robot):
        """Callback directo desde /robotN/odom — máxima tasa de Gazebo."""
        t = time.time() - self.start_time
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        theta = math.atan2(siny_cosp, cosy_cosp)

        with self._lock:
            if robot not in self.data:
                self.data[robot] = {"t": [], "x": [], "y": [], "theta": []}
            if robot not in self.initial_positions:
                self.initial_positions[robot] = (x, y)
            if self.first_msg_time is None:
                self.first_msg_time = time.time()
            self.data[robot]["t"].append(t)
            self.data[robot]["x"].append(x)
            self.data[robot]["y"].append(y)
            self.data[robot]["theta"].append(theta)

    # -------------------------------------------------

    def state_callback(self, msg):
        """Fallback: datos desde el relay /robot_states_plot."""
        robot = msg.robot_id
        t = time.time() - self.start_time

        with self._lock:
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

    def _vs_cb(self, msg):
        """Serie temporal del centroide de la estructura virtual."""
        t = time.time() - self.start_time
        with self._lock:
            self.vs_data['t'].append(t)
            self.vs_data['x'].append(msg.x)
            self.vs_data['y'].append(msg.y)
            self.vs_data['theta'].append(msg.theta)

    # -------------------------------------------------

    def _check_timing(self):
        """Solo vigila el tiempo — sin operaciones matplotlib."""
        with self._lock:
            if self.first_msg_time is None:
                return
            elapsed = time.time() - self.first_msg_time
            n_seen  = len(self.initial_positions)
            initial_saved = self.initial_saved
            final_saved   = self.final_saved

        if not initial_saved:
            all_seen = (self.n_robots_expected > 0 and n_seen >= self.n_robots_expected)
            time_ok  = (self.n_robots_expected <= 0 and elapsed >= self.t_initial)
            if all_seen or time_ok:
                with self._lock:
                    if not self.initial_saved:
                        self.initial_saved = True
                        do_initial = True
                    else:
                        do_initial = False
                if do_initial:
                    threading.Thread(target=self.save_initial, daemon=True).start()

        # Snapshot de formación
        if self.formation_snapshots:
            take = (self.last_snap_t is None and elapsed >= 1.0) or \
                   (self.last_snap_t is not None and
                    elapsed - self.last_snap_t >= self.snap_interval_s)
            if take:
                with self._lock:
                    snap_robots = {
                        r: (d['x'][-1], d['y'][-1], d['theta'][-1])
                        for r, d in self.data.items() if d['x']
                    }
                self.snapshots.append({'t': elapsed, 'robots': snap_robots})
                self.last_snap_t = elapsed

        if not final_saved and elapsed >= self.t_final:
            with self._lock:
                if not self.final_saved:
                    self.final_saved = True
                    do_final = True
                else:
                    do_final = False
            if do_final:
                threading.Thread(target=self._save_final_and_shutdown, daemon=True).start()

    def _save_final_and_shutdown(self):
        self.save_final()
        threading.Timer(2.0, self._shutdown_all).start()

    # -------------------------------------------------

    def save_initial(self):

        with self._lock:
            positions = dict(self.initial_positions)

        fig = Figure(figsize=(6, 6))
        FigureCanvasAgg(fig)
        ax = fig.add_subplot(1, 1, 1)

        for i, (robot, (x, y)) in enumerate(positions.items()):
            c = COLORS[i % len(COLORS)]
            ax.scatter(x, y, s=140, color=c, zorder=5, label=robot)
            ax.annotate(robot, (x, y),
                        textcoords="offset points", xytext=(8, 4), fontsize=10)

        ax.set_title("Initial positions")
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

    def _formation_ideals(self, snap_robots):
        """Devuelve {robot_id: (x_ideal, y_ideal)} según la V del líder en ese snapshot."""
        lid = self.formation_leader_id
        if lid not in snap_robots:
            return {}
        xL, yL, theta_L = snap_robots[lid]
        nf  = self.formation_n_followers
        av  = self.formation_angle_v
        d   = self.formation_d
        n_left  = nf // 2
        n_right = nf - n_left
        ideals  = {lid: (xL, yL)}
        idx = 1
        for k in range(1, n_left + 1):
            rx = -k * d * math.cos(theta_L + av)
            ry = -k * d * math.sin(theta_L + av)
            ideals[f'robot{idx}'] = (xL + rx, yL + ry)
            idx += 1
        for k in range(1, n_right + 1):
            rx = -k * d * math.cos(theta_L - av)
            ry = -k * d * math.sin(theta_L - av)
            ideals[f'robot{idx}'] = (xL + rx, yL + ry)
            idx += 1
        return ideals

    def save_final(self):

        with self._lock:
            data_snapshot = {r: {k: list(v) for k, v in d.items()}
                             for r, d in self.data.items()}

        if not data_snapshot:
            return

        self._save_raw(data_snapshot)

        if self.error_vs_virtual_structure:
            self._save_trajectory_and_error(data_snapshot)
        elif self.trajectory_only:
            self._save_trajectory_only(data_snapshot)
        else:
            self._save_triple(data_snapshot)

    def _save_raw(self, data_snapshot):
        """Datos crudos de la corrida para post-procesamiento (npz)."""
        arrays = {'labels': np.array(sorted(data_snapshot.keys()))}
        for robot, d in data_snapshot.items():
            for k in ('t', 'x', 'y', 'theta'):
                arrays[f'{robot}_{k}'] = np.array(d[k], dtype=float)
        path = os.path.join(self.save_dir, 'datos.npz')
        np.savez(path, **arrays)
        self.get_logger().info(f"Datos crudos guardados: {path}")

    def _save_trajectory_only(self, data_snapshot):

        fig = Figure(figsize=(5.5, 5.5))
        FigureCanvasAgg(fig)
        ax = fig.add_subplot(1, 1, 1)

        for i, (robot, d) in enumerate(data_snapshot.items()):
            c = COLORS[i % len(COLORS)]

            sort_idx = np.argsort(d["t"])
            x     = np.array(d["x"],     dtype=float)[sort_idx]
            y     = np.array(d["y"],     dtype=float)[sort_idx]
            t     = np.array(d["t"],     dtype=float)[sort_idx]
            theta = np.array(d["theta"], dtype=float)[sort_idx]

            _, uniq = np.unique(t, return_index=True)
            x, y, t, theta = x[uniq], y[uniq], t[uniq], theta[uniq]

            ax.plot(x, y, color=c, linewidth=2.0, zorder=2)
            ax.scatter(x[0], y[0], s=100, facecolors='white',
                       edgecolors=c, linewidths=2, zorder=4)
            ax.scatter(x[-1], y[-1], color=c, marker='*',
                       s=260, zorder=5, edgecolors='black', linewidths=0.5)
            self._draw_triangles(ax, x, y, theta, c)

        ax.set_title("Robot trajectory")
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.grid(True)
        ax.set_aspect('equal')
        fig.tight_layout()

        path = os.path.join(self.save_dir, "trajectory.pdf")
        fig.savefig(path, bbox_inches="tight")
        self.get_logger().info(f"Trayectoria guardada: {path}")

    def _save_trajectory_and_error(self, data_snapshot):
        """Dos gráficas: trayectorias (idéntica a _save_triple) y error
        euclidiano de cada robot respecto a su offset ideal dentro de la V,
        interpolando la pose de la estructura virtual en cada instante."""

        with self._lock:
            vs_t     = np.array(self.vs_data['t'],     dtype=float)
            vs_x     = np.array(self.vs_data['x'],     dtype=float)
            vs_y     = np.array(self.vs_data['y'],     dtype=float)
            vs_theta = np.array(self.vs_data['theta'], dtype=float)

        if len(vs_t) >= 2:
            sort_vs = np.argsort(vs_t)
            vs_t, vs_x, vs_y, vs_theta = (
                vs_t[sort_vs], vs_x[sort_vs], vs_y[sort_vs], vs_theta[sort_vs])
            _, uniq_vs = np.unique(vs_t, return_index=True)
            vs_t, vs_x, vs_y, vs_theta = (
                vs_t[uniq_vs], vs_x[uniq_vs], vs_y[uniq_vs], vs_theta[uniq_vs])
            have_vs = True
        else:
            have_vs = False
            self.get_logger().warn(
                "error_vs_virtual_structure=True pero no se recibió /virtual_structure "
                "— se omite la gráfica de error"
            )

        fig = Figure(figsize=(11, 5.2))
        FigureCanvasAgg(fig)
        axes = fig.subplots(1, 2)

        for i, (robot, d) in enumerate(sorted(data_snapshot.items())):
            c = COLORS[i % len(COLORS)]

            sort_idx = np.argsort(d["t"])
            x     = np.array(d["x"],     dtype=float)[sort_idx]
            y     = np.array(d["y"],     dtype=float)[sort_idx]
            t     = np.array(d["t"],     dtype=float)[sort_idx]
            theta = np.array(d["theta"], dtype=float)[sort_idx]

            _, uniq = np.unique(t, return_index=True)
            x, y, t, theta = x[uniq], y[uniq], t[uniq], theta[uniq]

            axes[0].plot(x, y, color=c, label=robot, linewidth=1.8, zorder=2)
            axes[0].scatter(x[0], y[0], s=90, facecolors='white',
                            edgecolors=c, linewidths=2, zorder=4)
            axes[0].scatter(x[-1], y[-1], color=c, marker='*',
                            s=220, zorder=5, edgecolors='black', linewidths=0.5)
            self._draw_triangles(axes[0], x, y, theta, c)

            if have_vs:
                try:
                    follower_index = int(robot.replace('robot', ''))
                except ValueError:
                    follower_index = 0

                vs_x_i  = np.interp(t, vs_t, vs_x)
                vs_y_i  = np.interp(t, vs_t, vs_y)
                vs_th_i = np.interp(t, vs_t, vs_theta)

                ox = np.empty_like(vs_th_i)
                oy = np.empty_like(vs_th_i)
                for j, th in enumerate(vs_th_i):
                    ox[j], oy[j] = _compute_offset(
                        follower_index, self.n_robots_expected, th,
                        self.formation_angle_v, self.formation_d)

                error = np.hypot(x - (vs_x_i + ox), y - (vs_y_i + oy))
                axes[1].plot(t, error, color=c, label=robot, linewidth=1.8)

        axes[0].set_title("Robot trajectories")
        axes[0].set_xlabel("x (m)")
        axes[0].set_ylabel("y (m)")
        axes[0].grid(True)
        axes[0].set_aspect('equal')
        axes[0].legend(fontsize=13)

        axes[1].set_title("Formation error evolution")
        axes[1].set_xlabel("Time (s)")
        axes[1].set_ylabel("Error (m)")
        axes[1].grid(True)
        axes[1].legend(fontsize=13)

        # Snapshots de formación sobre el panel de trayectorias — idéntico a _save_triple
        if self.formation_snapshots and self.snapshots:
            robot_color = {r: COLORS[i % len(COLORS)]
                           for i, r in enumerate(sorted(data_snapshot.keys()))}
            for snap_data in self.snapshots:
                snap   = snap_data['robots']
                ideals = self._formation_ideals(snap)
                for rid, (xa, ya, _) in snap.items():
                    c = robot_color.get(rid, 'gray')
                    axes[0].scatter(xa, ya, s=55, color=c, zorder=7, alpha=0.80)
                    if rid in ideals:
                        xi_pt, yi_pt = ideals[rid]
                        axes[0].scatter(xi_pt, yi_pt, s=55,
                                        facecolors='none', edgecolors=c,
                                        linewidths=1.8, zorder=7, alpha=0.80)
                        axes[0].plot([xa, xi_pt], [ya, yi_pt],
                                     color=c, linewidth=0.9,
                                     linestyle='--', alpha=0.45, zorder=6)

        fig.tight_layout()

        path = os.path.join(self.save_dir, "posiciones_finales.pdf")
        fig.savefig(path, bbox_inches="tight")
        self.get_logger().info(f"Trayectoria + error de formación guardados: {path}")

    def _save_triple(self, data_snapshot):

        fig = Figure(figsize=(15, 5))
        FigureCanvasAgg(fig)
        axes = fig.subplots(1, 3)

        for i, (robot, d) in enumerate(data_snapshot.items()):
            c = COLORS[i % len(COLORS)]

            sort_idx = np.argsort(d["t"])
            x     = np.array(d["x"],     dtype=float)[sort_idx]
            y     = np.array(d["y"],     dtype=float)[sort_idx]
            t     = np.array(d["t"],     dtype=float)[sort_idx]
            theta = np.array(d["theta"], dtype=float)[sort_idx]

            _, uniq = np.unique(t, return_index=True)
            x, y, t, theta = x[uniq], y[uniq], t[uniq], theta[uniq]

            axes[0].plot(x, y, color=c, label=robot, linewidth=1.8, zorder=2)
            axes[0].scatter(x[0], y[0], s=90, facecolors='white',
                            edgecolors=c, linewidths=2, zorder=4)
            axes[0].scatter(x[-1], y[-1], color=c, marker='*',
                            s=220, zorder=5, edgecolors='black', linewidths=0.5)
            self._draw_triangles(axes[0], x, y, theta, c)

            axes[1].plot(t, x, color=c, label=robot, linewidth=1.8)
            axes[2].plot(t, y, color=c, label=robot, linewidth=1.8)

        axes[0].set_title("Robot trajectories")
        axes[0].set_xlabel("x (m)")
        axes[0].set_ylabel("y (m)")
        axes[0].grid(True)
        axes[0].set_aspect('equal')
        axes[0].legend(fontsize=8)

        axes[1].set_title("X Evolution")
        axes[1].set_xlabel("Time (s)")
        axes[1].set_ylabel("x (m)")
        axes[1].grid(True)
        axes[1].legend(fontsize=8)

        axes[2].set_title("Y Evolution")
        axes[2].set_xlabel("Time (s)")
        axes[2].set_ylabel("y (m)")
        axes[2].grid(True)
        axes[2].legend(fontsize=8)

        # Snapshots de formación: círculo relleno=real, círculo hueco=ideal
        if self.formation_snapshots and self.snapshots:
            robot_color = {r: COLORS[i % len(COLORS)]
                           for i, r in enumerate(sorted(data_snapshot.keys()))}
            for snap_data in self.snapshots:
                snap    = snap_data['robots']
                ideals  = self._formation_ideals(snap)
                # Conectar todos los ideales con línea gris discontinua (forma de la V)
                ideal_pts = list(ideals.values())
                if len(ideal_pts) > 1:
                    ix = [p[0] for p in ideal_pts]
                    iy = [p[1] for p in ideal_pts]
                for rid, (xa, ya, _) in snap.items():
                    c = robot_color.get(rid, 'gray')
                    # Posición real
                    axes[0].scatter(xa, ya, s=55, color=c,
                                    zorder=7, alpha=0.80)
                    # Posición ideal
                    if rid in ideals:
                        xi_pt, yi_pt = ideals[rid]
                        axes[0].scatter(xi_pt, yi_pt, s=55,
                                        facecolors='none', edgecolors=c,
                                        linewidths=1.8, zorder=7, alpha=0.80)
                        # Línea de error (real → ideal)
                        axes[0].plot([xa, xi_pt], [ya, yi_pt],
                                     color=c, linewidth=0.9,
                                     linestyle='--', alpha=0.45, zorder=6)

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
        with self._lock:
            if self.final_saved:
                return
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
            threading.Thread(target=node.save_final, daemon=True).start()

    hilo = threading.Thread(target=esperar_enter, daemon=True)
    hilo.start()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.save_figures()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
