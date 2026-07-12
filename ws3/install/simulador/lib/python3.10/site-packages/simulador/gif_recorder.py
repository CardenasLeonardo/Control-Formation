import os
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import imageio.v2 as imageio


class GifRecorder(Node):
    """
    Frames: se escriben a disco de inmediato en el callback (no se bufferean en
    memoria) para que sobrevivan al auto-shutdown (pkill -9) aunque la escritura
    de una simulación larga tome más que el margen de 2s del shutdown.
    El GIF es solo un preview — se arma al final releyendo los PNG ya guardados.
    """

    def __init__(self):

        super().__init__('gif_recorder')

        self.declare_parameter('image_topic',   '/overhead_camera/image_raw')
        self.declare_parameter('save_dir',      '')
        self.declare_parameter('gif_name',      'simulacion.gif')
        self.declare_parameter('t_final',       30.0)
        self.declare_parameter('fps',           10.0)
        self.declare_parameter('save_frames',   True)
        self.declare_parameter('frame_prefix',  'frame')
        # Parada por meta alcanzada, en vez de por tiempo fijo
        self.declare_parameter('stop_mode',     'time')   # 'time' | 'goal'
        self.declare_parameter('goal_topic',    '')        # ej. '/robot0/final_goal_reached'
        self.declare_parameter('stop_delay',    2.0)       # s tras la meta antes de finalizar
        self.declare_parameter('auto_shutdown', False)     # matar la simulación al finalizar
        # El .gif es solo preview — se limita para no reventar RAM/tiempo en
        # simulaciones largas (miles de frames). Los PNG en frames/ no se tocan.
        self.declare_parameter('gif_max_frames', 150)
        self.declare_parameter('gif_width',      480)      # 0 = sin reescalar

        self.image_topic   = str(self.get_parameter('image_topic').value)
        self.save_dir      = str(self.get_parameter('save_dir').value) or 'gifs'
        self.gif_name      = str(self.get_parameter('gif_name').value)
        self.t_final       = float(self.get_parameter('t_final').value)
        self.fps           = float(self.get_parameter('fps').value)
        self.save_frames   = bool(self.get_parameter('save_frames').value)
        self.frame_prefix  = str(self.get_parameter('frame_prefix').value)
        self.stop_mode     = str(self.get_parameter('stop_mode').value)
        self.goal_topic    = str(self.get_parameter('goal_topic').value)
        self.stop_delay    = float(self.get_parameter('stop_delay').value)
        self.auto_shutdown = bool(self.get_parameter('auto_shutdown').value)
        self.gif_max_frames = int(self.get_parameter('gif_max_frames').value)
        self.gif_width      = int(self.get_parameter('gif_width').value)

        self.gifs_dir   = os.path.join(self.save_dir, 'gifs')
        self.frames_dir = os.path.join(self.save_dir, 'frames')
        os.makedirs(self.gifs_dir, exist_ok=True)
        os.makedirs(self.frames_dir, exist_ok=True)

        self.bridge         = CvBridge()
        self.frames_buffer  = []   # solo se usa si save_frames=False
        self._lock          = threading.Lock()
        self._frame_count   = 0
        self._finalized     = False
        self._goal_seen     = False
        self._min_period    = 1.0 / self.fps
        self._last_capture  = -self._min_period
        self.start_time     = time.time()

        self.create_subscription(Image, self.image_topic, self._image_cb, 10)
        self.timer = self.create_timer(0.5, self._check_timing)

        if self.stop_mode == 'goal' and self.goal_topic:
            self.create_subscription(Bool, self.goal_topic, self._goal_reached_cb, 10)
            self.get_logger().info(
                f"GIF Recorder — suscrito a {self.image_topic}, "
                f"guardando frames en vivo en {self.frames_dir}, "
                f"cierre {self.stop_delay}s después de {self.goal_topic} "
                f"(t_final={self.t_final}s como respaldo) a {self.fps} fps"
            )
        else:
            self.get_logger().info(
                f"GIF Recorder — suscrito a {self.image_topic}, "
                f"guardando frames en vivo en {self.frames_dir}, "
                f"cierre en t={self.t_final}s a {self.fps} fps"
            )

    def _image_cb(self, msg):
        with self._lock:
            if self._finalized:
                return

        now = time.time() - self.start_time
        if now - self._last_capture < self._min_period:
            return
        self._last_capture = now

        if self.save_frames:
            frame_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self._lock:
                idx = self._frame_count
                self._frame_count += 1
            cv2.imwrite(
                os.path.join(self.frames_dir, f"{self.frame_prefix}-{idx}.png"),
                frame_bgr
            )
        else:
            frame_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            with self._lock:
                self.frames_buffer.append(frame_rgb)

    def _check_timing(self):
        elapsed = time.time() - self.start_time
        with self._lock:
            finalized = self._finalized
        if not finalized and elapsed >= self.t_final:
            with self._lock:
                self._finalized = True
            self.get_logger().warn(
                f"t_final={self.t_final}s alcanzado sin señal de meta — cerrando por respaldo"
            )
            threading.Thread(target=self.finalize, daemon=True).start()

    def _goal_reached_cb(self, msg):
        if not msg.data:
            return
        with self._lock:
            if self._goal_seen:
                return
            self._goal_seen = True
        self.get_logger().info(
            f"Meta final recibida en {self.goal_topic} — cerrando en {self.stop_delay}s"
        )
        threading.Timer(self.stop_delay, self._trigger_finalize).start()

    def _trigger_finalize(self):
        with self._lock:
            if self._finalized:
                return
            self._finalized = True
        self.finalize()

    def finalize(self):
        try:
            if self.save_frames:
                with self._lock:
                    n_frames = self._frame_count
                if n_frames == 0:
                    self.get_logger().warn("No se capturaron frames — revisa que /overhead_camera/image_raw esté publicando")
                else:
                    self.get_logger().info(
                        f"Frames guardados en: {self.frames_dir} "
                        f"({self.frame_prefix}-0.png .. {self.frame_prefix}-{n_frames - 1}.png, {n_frames} frames)"
                    )
                    self._build_gif_from_disk(n_frames)
            else:
                with self._lock:
                    frames = list(self.frames_buffer)
                if not frames:
                    self.get_logger().warn("No se capturaron frames — revisa que /overhead_camera/image_raw esté publicando")
                else:
                    self._save_gif(frames)
        except Exception as e:
            # Los frames en disco ya están a salvo — esto solo afecta el .gif de preview
            self.get_logger().error(f"No se pudo armar el GIF de preview: {e!r}")
        finally:
            if self.auto_shutdown:
                threading.Timer(1.0, self._shutdown_all).start()

    def _shutdown_all(self):
        self.get_logger().info("Cerrando simulación automáticamente (auto_shutdown)...")
        subprocess.Popen(
            'pkill -9 -f gzserver; pkill -9 -f gzclient; '
            'pkill -9 -f "ros2 launch"; pkill -9 -f navigate_waypoints_pva; '
            'pkill -9 -f navigate_individual; pkill -9 -f consenso_node; '
            'pkill -9 -f sphere_demo; pkill -9 -f vs_node; '
            'pkill -9 -f "simulador/lib/simulador/aire"; '
            'pkill -9 -f states_plotter; pkill -9 -f pva_plotter; '
            'pkill -9 -f camera_zoom; '
            'pkill -9 -f gif_recorder',
            shell=True
        )

    def _build_gif_from_disk(self, n_frames):
        indices = range(n_frames)
        if n_frames > self.gif_max_frames:
            step = n_frames / self.gif_max_frames
            indices = sorted({int(i * step) for i in range(self.gif_max_frames)})
            self.get_logger().info(
                f"GIF preview: submuestreando {n_frames} frames a {len(indices)} "
                f"(gif_max_frames={self.gif_max_frames}) — el set completo en "
                f"{self.frames_dir} no se toca"
            )

        frames = []
        for i in indices:
            frame_path = os.path.join(self.frames_dir, f"{self.frame_prefix}-{i}.png")
            frame_bgr = cv2.imread(frame_path)
            if frame_bgr is None:
                continue
            if self.gif_width > 0:
                h, w = frame_bgr.shape[:2]
                new_h = max(1, int(h * self.gif_width / w))
                frame_bgr = cv2.resize(frame_bgr, (self.gif_width, new_h), interpolation=cv2.INTER_AREA)
            frames.append(cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB))
        self._save_gif(frames)

    def _save_gif(self, frames):
        if not frames:
            self.get_logger().warn("No se pudo armar el GIF: no hay frames legibles")
            return
        path = os.path.join(self.gifs_dir, self.gif_name)
        imageio.mimsave(path, frames, duration=1.0 / self.fps)
        self.get_logger().info(f"GIF guardado: {path} ({len(frames)} frames)")


def main(args=None):

    rclpy.init(args=args)
    node = GifRecorder()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        with node._lock:
            already_finalized = node._finalized
            node._finalized = True
        if not already_finalized:
            node.finalize()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
