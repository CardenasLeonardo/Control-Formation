import os
import math
import signal
import threading
import time
import rclpy
from rclpy.node import Node

import numpy as np
import matplotlib.pyplot as plt
import imageio.v2 as imageio
from std_msgs.msg import Bool

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
        # Grabación de la evolución del polígono como GIF
        # (mismo patrón que gif_recorder.py, pero para la figura
        #  matplotlib en vez de la cámara de Gazebo)
        # ---------------------------------------------
        self.declare_parameter('record_gif',   False)
        self.declare_parameter('save_dir',     '')
        self.declare_parameter('gif_name',     'pva_evolution.gif')
        self.declare_parameter('record_fps',   5.0)
        self.declare_parameter('stop_mode',    'time')   # 'time' | 'goal'
        self.declare_parameter('goal_topic',   '')
        self.declare_parameter('stop_delay',   1.5)
        self.declare_parameter('t_final',      60.0)
        self.declare_parameter('gif_max_frames', 150)
        self.declare_parameter('gif_width',      480)

        self.record_gif    = bool(self.get_parameter('record_gif').value)
        self.gif_name       = str(self.get_parameter('gif_name').value)
        self.record_fps     = float(self.get_parameter('record_fps').value)
        self.stop_mode       = str(self.get_parameter('stop_mode').value)
        self.goal_topic       = str(self.get_parameter('goal_topic').value)
        self.stop_delay       = float(self.get_parameter('stop_delay').value)
        self.t_final           = float(self.get_parameter('t_final').value)
        self.gif_max_frames = int(self.get_parameter('gif_max_frames').value)
        self.gif_width        = int(self.get_parameter('gif_width').value)

        if self.record_gif:
            save_dir = str(self.get_parameter('save_dir').value) or 'figures_pva'
            # Subcarpeta propia para no chocar con los frames de gif_recorder
            # (misma convención de nombres 'frame-{n}.png' en el mismo save_dir)
            self.gifs_dir   = os.path.join(save_dir, 'pva_evolution', 'gifs')
            self.frames_dir = os.path.join(save_dir, 'pva_evolution', 'frames')
            os.makedirs(self.gifs_dir, exist_ok=True)
            os.makedirs(self.frames_dir, exist_ok=True)

            self._rec_lock       = threading.Lock()
            self._frame_count    = 0
            self._finalized_rec  = False
            self._goal_seen_rec  = False
            self._min_period_rec = 1.0 / self.record_fps
            self._last_capture   = -self._min_period_rec
            self._rec_start_time = time.time()

            if self.stop_mode == 'goal' and self.goal_topic:
                self.create_subscription(Bool, self.goal_topic, self._goal_reached_cb, 10)

            self.get_logger().info(
                f"PVA Plotter — grabando evolución en {self.frames_dir} "
                f"a {self.record_fps} fps (stop_mode={self.stop_mode})"
            )

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

        # SIGUSR1 → guardar figuras sin detener el nodo
        signal.signal(signal.SIGUSR1, lambda sig, frame: self.save_figures())

        self.get_logger().info("PVA Plotter iniciado  |  kill -USR1 <pid> para guardar PDF")

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
            "w_star": float(msg.w_star),
            "is_deadlock":  bool(msg.is_deadlock),
            "search_dir":   int(msg.search_dir),
            "has_edge":     bool(msg.has_edge),
            "edge_sel":     (float(msg.edge_v_sel),   float(msg.edge_w_sel)),
            "edge_other":   (float(msg.edge_v_other), float(msg.edge_w_other)),
            "v_z":          float(msg.v_z),
            "v_z_previous": float(msg.v_z_previous),
        }

    # --------------------------------------------------
    # CONSTRUIR LAYOUT DINÁMICO
    # --------------------------------------------------

    def build_layout(self):

        n = len(self.data)

        cols = math.ceil(math.sqrt(n))
        rows = math.ceil(n / cols)

        # constrained_layout en vez de llamar tight_layout() en cada
        # frame: tight_layout() recalcula el layout COMPLETO de la figura
        # cada vez que se llama (caro), mientras que constrained_layout
        # lo resuelve una sola vez por cambios reales de tamaño/artistas,
        # mucho más barato en el loop de refresco.
        self.fig_main, axs = plt.subplots(
            rows, cols, figsize=(5*cols, 4*rows), constrained_layout=True
        )

        axs = np.atleast_1d(axs).flatten()

        self.axes = {}

        for ax, robot in zip(axs, self.data.keys()):
            self.axes[robot] = ax

        for ax in axs[len(self.data):]:
            ax.axis("off")

        plt.show(block=False)

    # --------------------------------------------------
    # CALCULAR POLÍGONO FACTIBLE
    # --------------------------------------------------

    def compute_polygon(self, A, B, C):
        # Vectorizado con numpy en vez del doble loop de Python: para n
        # restricciones son n(n-1)/2 pares de rectas — con n_rays_used
        # alto (72+box) esto son miles de intersecciones evaluadas en
        # Python puro cada frame, que es buena parte del costo de refresco
        # de la gráfica. Aquí se calculan TODAS las intersecciones y la
        # factibilidad de una sola vez con operaciones de arreglo.
        n = len(A)
        i, j = np.triu_indices(n, k=1)

        det = A[i] * B[j] - A[j] * B[i]
        valid = np.abs(det) > 1e-9
        i, j, det = i[valid], j[valid], det[valid]
        if len(det) == 0:
            return None

        v = (C[i] * B[j] - C[j] * B[i]) / det
        w = (A[i] * C[j] - A[j] * C[i]) / det

        # A @ v + B @ w <= C + tol para TODAS las restricciones, por cada
        # punto candidato (broadcasting: (n_constraints, n_puntos))
        feasible = np.all(
            A[:, None] * v[None, :] + B[:, None] * w[None, :] <= C[:, None] + 1e-7,
            axis=0,
        )
        v, w = v[feasible], w[feasible]

        if len(v) == 0:
            return None

        points = np.column_stack([v, w])
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

    _DEADLOCK_COLOR = {True: "tab:orange", False: "tab:green"}

    def draw_robot_pva(self, ax, robot, d):

        ax.cla()

        is_deadlock  = d.get("is_deadlock", False)
        border_color = self._DEADLOCK_COLOR[is_deadlock]

        ax.set_title(f"PVA - {robot}")
        ax.set_xlabel(r"Linear velocity $v$ (m/s)")
        ax.set_ylabel(r"Angular velocity $\omega$ (rad/s)")
        ax.grid(True)

        # Contorno de los ejes: naranja si Deadlock detectó atasco,
        # verde si el robot se mueve con normalidad.
        for spine in ax.spines.values():
            spine.set_edgecolor(border_color)
            spine.set_linewidth(3.0)

        ax.axhline(0.0, linestyle="--", color="gray", linewidth=1.0)
        ax.axvline(0.0, linestyle="--", color="gray", linewidth=1.0)

        # Los límites de velocidad ya vienen incluidos en a/b/c (ver
        # control_nodes/algorithms/pva.py, build_constraints), no hace
        # falta agregarlos otra vez aquí.
        A = d["a"]
        B = d["b"]
        C = d["c"]

        polygon = self.compute_polygon(A, B, C)

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

            ax.scatter(
                polygon[:,0],
                polygon[:,1],
                color="black",
                s=25,
                zorder=5,
                label="Vertices"
            )

        # Arista seleccionada por Deadlock (estrategia de bordeo): solo
        # se dibuja cuando hay atasco Y la arista quedó bien definida
        # (dos vértices sobre la misma recta). Vértice elegido en verde
        # claro, el otro extremo de la arista en verde fuerte, y la
        # dirección de bordeo (sR/sL) como texto.
        if is_deadlock and d.get("has_edge", False):
            v_sel,   w_sel   = d["edge_sel"]
            v_other, w_other = d["edge_other"]

            ax.plot(
                [v_sel, v_other], [w_sel, w_other],
                color="red", linewidth=2.5, zorder=6,
                label="Selected constraint",
            )
            ax.scatter(
                [v_other], [w_other],
                color="darkgreen", s=90, zorder=7,
                label="Edge vertex (other)",
            )
            ax.scatter(
                [v_sel], [w_sel],
                color="lightgreen", edgecolor="darkgreen", linewidth=1.2,
                s=90, zorder=8,
                label="Edge vertex (selected)",
            )

            direction_label = "sR" if d.get("search_dir", 1) == 1 else "sL"
            ax.text(
                0.98, 0.98, f"dir = {direction_label}",
                transform=ax.transAxes, ha="right", va="top",
                fontsize=10, fontweight="bold", color="white",
                bbox=dict(facecolor="red", edgecolor="none",
                          boxstyle="round,pad=0.3", alpha=0.9),
                zorder=10,
            )

        # V(z) actual y V(z) capturado al entrar en deadlock.
        v_z          = d.get("v_z", 0.0)
        v_z_previous = d.get("v_z_previous", 0.0)
        ax.text(
            0.02, 0.86,
            f"$V(z)$ = {v_z:.4f}\n$V(z)_{{prev}}$ = {v_z_previous:.4f}",
            transform=ax.transAxes, ha="left", va="top",
            fontsize=9, color="black",
            bbox=dict(facecolor="white", edgecolor="gray",
                      boxstyle="round,pad=0.3", alpha=0.85),
            zorder=10,
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

        # Círculo punteado centrado en u_ref con radio = distancia a u*
        radius = math.sqrt((d["v_star"] - d["v_goal"])**2 +
                           (d["w_star"] - d["w_goal"])**2)
        if radius > 1e-6:
            theta_c = np.linspace(0, 2 * np.pi, 200)
            ax.plot(
                d["v_goal"] + radius * np.cos(theta_c),
                d["w_goal"] + radius * np.sin(theta_c),
                linestyle='--',
                linewidth=1.0,
                color='gray',
                alpha=0.7,
            )

        ax.set_xlim(-1.0, 1.0)
        ax.set_ylim(-1.0, 1.0)
        ax.set_aspect('equal', adjustable='box')

        ax.legend()

    # --------------------------------------------------
    # UPDATE PLOT
    # --------------------------------------------------

    def update_plot(self):

        if not self.data:
            return

        if self.fig_main is None or len(self.axes) != len(self.data):
            if self.fig_main is not None:
                plt.close(self.fig_main)
            self.build_layout()

        for robot, d in self.data.items():

            ax = self.axes[robot]

            self.draw_robot_pva(ax, robot, d)

        self.fig_main.canvas.draw_idle()
        self.fig_main.canvas.flush_events()

        if self.record_gif:
            self._capture_frame()

    # --------------------------------------------------
    # GRABACIÓN DE FRAMES (evolución del polígono)
    # --------------------------------------------------

    def _capture_frame(self):
        with self._rec_lock:
            if self._finalized_rec:
                return
        now = time.time() - self._rec_start_time
        if now - self._last_capture < self._min_period_rec:
            return
        self._last_capture = now

        with self._rec_lock:
            idx = self._frame_count
            self._frame_count += 1

        self.fig_main.savefig(
            os.path.join(self.frames_dir, f"frame-{idx}.png"),
            dpi=100
        )

        if self.stop_mode == 'time' and now >= self.t_final:
            with self._rec_lock:
                if not self._finalized_rec:
                    self._finalized_rec = True
                    threading.Thread(target=self._finalize_recording, daemon=True).start()

    def _goal_reached_cb(self, msg):
        if not msg.data:
            return
        with self._rec_lock:
            if self._goal_seen_rec:
                return
            self._goal_seen_rec = True
        self.get_logger().info(
            f"PVA Plotter — meta recibida en {self.goal_topic}, cerrando grabación en {self.stop_delay}s"
        )
        threading.Timer(self.stop_delay, self._trigger_finalize_recording).start()

    def _trigger_finalize_recording(self):
        with self._rec_lock:
            if self._finalized_rec:
                return
            self._finalized_rec = True
        self._finalize_recording()

    def _finalize_recording(self):
        with self._rec_lock:
            n_frames = self._frame_count
        if n_frames == 0:
            self.get_logger().warn("PVA Plotter — no se capturaron frames de la gráfica")
            return

        indices = range(n_frames)
        if n_frames > self.gif_max_frames:
            step = n_frames / self.gif_max_frames
            indices = sorted({int(i * step) for i in range(self.gif_max_frames)})

        frames = []
        for i in indices:
            path = os.path.join(self.frames_dir, f"frame-{i}.png")
            if not os.path.exists(path):
                continue
            frames.append(imageio.imread(path))

        if not frames:
            self.get_logger().warn("PVA Plotter — no se pudo armar el GIF: no hay frames legibles")
            return

        gif_path = os.path.join(self.gifs_dir, self.gif_name)
        imageio.mimsave(gif_path, frames, duration=1.0 / self.record_fps)
        self.get_logger().info(
            f"PVA Plotter — GIF de evolución guardado: {gif_path} ({len(frames)} frames, de {n_frames} capturados)"
        )

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

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        while rclpy.ok():
            node.update_plot()
            # 1/7s era un límite artificial de refresco; con
            # tight_layout() fuera del loop y compute_polygon vectorizado
            # el render ya es mucho más barato — se sube el techo a ~30fps
            # (más no ayuda: por debajo de ~33ms domina el costo real de
            # redibujar la figura, no la espera de plt.pause).
            plt.pause(1.0 / 30.0)

    except KeyboardInterrupt:
        node.get_logger().info("Guardando figuras...")
        node.save_figures()
        if node.record_gif:
            node._finalize_recording()

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()