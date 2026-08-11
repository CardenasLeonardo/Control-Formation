import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


# ------------------------------------------------------------------
# Nodo mínimo para material de presentación: guarda UNA sola imagen
# fija de la cámara cenital (no un GIF ni una serie de frames) y se
# apaga. Útil para capturas estáticas de escenarios simples (ej. el
# cubo de spawn_cube_demo) donde no hace falta animación.
# ------------------------------------------------------------------

class SnapshotCameraNode(Node):

    def __init__(self):
        super().__init__('snapshot_camera')

        self.declare_parameter('image_topic', '/overhead_camera/image_raw')
        self.declare_parameter('save_dir',    'figuras')
        self.declare_parameter('file_name',   'snapshot.png')
        self.declare_parameter('delay',       2.0)   # s de espera antes de capturar

        self.image_topic = str(self.get_parameter('image_topic').value)
        self.save_dir    = str(self.get_parameter('save_dir').value)
        self.file_name   = str(self.get_parameter('file_name').value)
        self.delay       = float(self.get_parameter('delay').value)

        os.makedirs(self.save_dir, exist_ok=True)

        self.bridge   = CvBridge()
        self._elapsed = 0.0
        self._saved   = False

        self.create_subscription(Image, self.image_topic, self._image_cb, 10)
        self.create_timer(0.1, self._tick)

        self.get_logger().info(
            f"Snapshot Camera — suscrito a {self.image_topic}, "
            f"capturará tras {self.delay}s en {self.save_dir}/{self.file_name}"
        )

    def _tick(self):
        self._elapsed += 0.1

    def _image_cb(self, msg):
        if self._saved or self._elapsed < self.delay:
            return
        self._saved = True

        frame_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        path = os.path.join(self.save_dir, self.file_name)
        cv2.imwrite(path, frame_bgr)
        self.get_logger().info(f"Snapshot guardado: {path}")

        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = SnapshotCameraNode()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
