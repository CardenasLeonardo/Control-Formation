import os
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import imageio.v2 as imageio
import cv2


class GifCombiner(Node):
    """
    Combina, frame a frame, los PNG ya escritos en disco por gif_recorder
    (cámara cenital de Gazebo) y por pva_plotter (evolución del polígono
    FVP) en un tercer GIF: cada frame del resultado es la unión lado a lado
    (hstack) del frame i de la cámara con el frame i de la gráfica PVA, así
    que el GIF final es más ancho, no más alto — misma duración, ambos
    sincronizados por índice.

    Sincronización: ambos nodos capturan al mismo fps y arrancan en el
    mismo TimerAction del launch, así que el frame i de uno corresponde al
    mismo instante (± un frame) que el frame i del otro. Se combina hasta
    min(n_frames_camera, n_frames_pva) — el nodo que capturó menos frames
    manda la duración final.

    Este nodo solo LEE PNGs ya guardados por los otros dos (nunca depende
    de que sigan vivos), así que puede correr con auto_shutdown=True en
    gif_recorder siempre que su propio stop_delay sea MENOR: debe terminar
    de escribir el GIF combinado antes de que el pkill -9 de gif_recorder
    tire la simulación.
    """

    def __init__(self):

        super().__init__('gif_combiner')

        self.declare_parameter('save_dir',          '')
        self.declare_parameter('camera_frames_dir', '')   # default: save_dir/frames
        self.declare_parameter('camera_frame_prefix', 'frame')
        self.declare_parameter('pva_frames_dir',    '')   # default: save_dir/pva_evolution/frames
        self.declare_parameter('gif_name',          'combined.gif')
        self.declare_parameter('fps',               10.0)
        self.declare_parameter('gif_max_frames',    150)

        self.declare_parameter('stop_mode',   'time')   # 'time' | 'goal'
        self.declare_parameter('goal_topic',  '')
        self.declare_parameter('stop_delay',  1.0)      # DEBE ser < stop_delay de gif_recorder
        self.declare_parameter('t_final',     90.0)

        save_dir = self.get_parameter('save_dir').value or 'figuras/gif_combiner'
        self.save_dir = save_dir
        self.gifs_dir = os.path.join(save_dir, 'gifs')
        os.makedirs(self.gifs_dir, exist_ok=True)

        cam_dir = self.get_parameter('camera_frames_dir').value
        self.camera_frames_dir = cam_dir or os.path.join(save_dir, 'frames')
        self.camera_frame_prefix = str(self.get_parameter('camera_frame_prefix').value)

        pva_dir = self.get_parameter('pva_frames_dir').value
        self.pva_frames_dir = pva_dir or os.path.join(save_dir, 'pva_evolution', 'frames')

        self.gif_name       = str(self.get_parameter('gif_name').value)
        self.fps             = float(self.get_parameter('fps').value)
        self.gif_max_frames  = int(self.get_parameter('gif_max_frames').value)

        self.stop_mode  = str(self.get_parameter('stop_mode').value)
        self.goal_topic = str(self.get_parameter('goal_topic').value)
        self.stop_delay = float(self.get_parameter('stop_delay').value)
        self.t_final    = float(self.get_parameter('t_final').value)

        self._lock       = threading.Lock()
        self._finalized   = False
        self._goal_seen    = False
        self._start_time  = time.time()

        if self.stop_mode == 'goal' and self.goal_topic:
            self.create_subscription(Bool, self.goal_topic, self._goal_reached_cb, 10)
            self.get_logger().info(
                f"GIF Combiner — esperando {self.goal_topic}, "
                f"combinará {self.stop_delay}s después (t_final={self.t_final}s como respaldo)"
            )
        else:
            self.create_timer(self.t_final, self._trigger_finalize)
            self.get_logger().info(f"GIF Combiner — combinará a t={self.t_final}s")

        # Respaldo de seguridad incondicional, por si stop_mode='goal' pero
        # la meta nunca llega (deadlock real) — no depende del timer de arriba.
        threading.Timer(self.t_final, self._trigger_finalize).start()

    # --------------------------------------------------

    def _goal_reached_cb(self, msg):
        if not msg.data:
            return
        with self._lock:
            if self._goal_seen:
                return
            self._goal_seen = True
        self.get_logger().info(
            f"GIF Combiner — meta recibida, combinando en {self.stop_delay}s"
        )
        threading.Timer(self.stop_delay, self._trigger_finalize).start()

    def _trigger_finalize(self):
        with self._lock:
            if self._finalized:
                return
            self._finalized = True
        self._combine()

    # --------------------------------------------------

    def _combine(self):
        n_cam = self._count_frames(self.camera_frames_dir, self.camera_frame_prefix)
        n_pva = self._count_frames(self.pva_frames_dir, 'frame')

        if n_cam == 0 or n_pva == 0:
            self.get_logger().warn(
                f"GIF Combiner — no hay frames para combinar "
                f"(cámara={n_cam}, pva={n_pva}); revisa que ambos nodos "
                f"hayan terminado de escribir en disco"
            )
            return

        n = min(n_cam, n_pva)
        indices = range(n)
        if n > self.gif_max_frames:
            step = n / self.gif_max_frames
            indices = sorted({int(i * step) for i in range(self.gif_max_frames)})

        frames = []
        for i in indices:
            cam_path = os.path.join(self.camera_frames_dir, f"{self.camera_frame_prefix}-{i}.png")
            pva_path = os.path.join(self.pva_frames_dir, f"frame-{i}.png")

            cam = cv2.imread(cam_path)
            pva = cv2.imread(pva_path)
            if cam is None or pva is None:
                continue

            # Misma altura antes de pegar lado a lado — se escala el más
            # bajo de los dos, sin recortar contenido.
            h = max(cam.shape[0], pva.shape[0])
            cam = self._resize_to_height(cam, h)
            pva = self._resize_to_height(pva, h)

            combined = np.hstack([cam, pva])
            frames.append(cv2.cvtColor(combined, cv2.COLOR_BGR2RGB))

        if not frames:
            self.get_logger().warn("GIF Combiner — no se pudo armar el GIF: no hay pares de frames legibles")
            return

        path = os.path.join(self.gifs_dir, self.gif_name)
        imageio.mimsave(path, frames, duration=1.0 / self.fps)
        self.get_logger().info(
            f"GIF combinado guardado: {path} "
            f"({len(frames)} frames, cámara={n_cam} pva={n_pva})"
        )

    @staticmethod
    def _resize_to_height(img, h):
        if img.shape[0] == h:
            return img
        new_w = max(1, int(img.shape[1] * h / img.shape[0]))
        return cv2.resize(img, (new_w, h), interpolation=cv2.INTER_AREA)

    @staticmethod
    def _count_frames(frames_dir, prefix):
        if not os.path.isdir(frames_dir):
            return 0
        n = 0
        while os.path.exists(os.path.join(frames_dir, f"{prefix}-{n}.png")):
            n += 1
        return n


def main(args=None):
    rclpy.init(args=args)
    node = GifCombiner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
