import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import SetEntityState


class CameraZoom(Node):
    """
    Acerca la cámara cenital (baja su altura y la centra en el robot) cuando
    éste entra en el radio de un obstáculo, y la regresa a la vista amplia
    cuando se aleja. Da la impresión de "corte" de cámara antes/durante/después
    de esquivar cada pilar.
    """

    def __init__(self):

        super().__init__('camera_zoom')

        self.declare_parameter('entity_name',  'overhead_camera')
        self.declare_parameter('odom_topic',   '/robot0/odom')
        self.declare_parameter('obstacles',    [1.0, 2.0, 2.0, 1.0, 3.0, 2.0, 2.0, 3.0])
        self.declare_parameter('wide_z',       7.0)
        self.declare_parameter('wide_x',       2.0)
        self.declare_parameter('wide_y',       2.0)
        self.declare_parameter('zoom_z',       2.5)
        self.declare_parameter('zoom_radius',  1.8)
        self.declare_parameter('update_rate',  5.0)

        self.entity_name = str(self.get_parameter('entity_name').value)
        self.wide_z       = float(self.get_parameter('wide_z').value)
        self.wide_x       = float(self.get_parameter('wide_x').value)
        self.wide_y       = float(self.get_parameter('wide_y').value)
        self.zoom_z       = float(self.get_parameter('zoom_z').value)
        self.zoom_radius  = float(self.get_parameter('zoom_radius').value)

        raw = list(self.get_parameter('obstacles').value)
        self.obstacles = [(raw[i], raw[i + 1]) for i in range(0, len(raw) - 1, 2)]

        self.declare_parameter('zoom_once', True)
        self.zoom_once = bool(self.get_parameter('zoom_once').value)

        self.x = None
        self.y = None
        self._last_state = None
        self._zoom_used     = False  # ya se gastó el único zoom permitido
        self._in_zoom_episode = False  # sigue dentro del acercamiento actual

        self.cli = self.create_client(SetEntityState, '/gazebo/set_entity_state')

        odom_topic = str(self.get_parameter('odom_topic').value)
        self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)

        rate = float(self.get_parameter('update_rate').value)
        self.create_timer(1.0 / rate, self._update)

        self.get_logger().info(
            f"Camera Zoom — vigilando {odom_topic}, radio de acercamiento "
            f"{self.zoom_radius} m, {len(self.obstacles)} obstáculos"
        )

    def _odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

    def _update(self):
        if self.x is None:
            self.get_logger().warn("Camera Zoom — sin odometría todavía", throttle_duration_sec=2.0)
            return
        if not self.cli.service_is_ready():
            self.get_logger().warn("Camera Zoom — servicio /gazebo/set_entity_state no listo", throttle_duration_sec=2.0)
            return

        d_min = min(math.hypot(self.x - ox, self.y - oy) for ox, oy in self.obstacles)
        near_obstacle = d_min < self.zoom_radius

        # zoom_once=True: se permite un solo ciclo de zoom-in/zoom-out (el
        # primer acercamiento). Una vez que ese ciclo termina (el robot se
        # aleja de nuevo), los acercamientos posteriores a cualquier
        # obstáculo ya no disparan zoom — la cámara se queda en vista amplia.
        if self.zoom_once:
            if near_obstacle:
                if self._zoom_used and not self._in_zoom_episode:
                    near_obstacle = False
                else:
                    self._zoom_used = True
                    self._in_zoom_episode = True
            else:
                self._in_zoom_episode = False

        if near_obstacle:
            cx, cy, cz = self.x, self.y, self.zoom_z
        else:
            cx, cy, cz = self.wide_x, self.wide_y, self.wide_z

        self.get_logger().info(
            f"Camera Zoom — robot=({self.x:.2f},{self.y:.2f}) d_min={d_min:.2f} "
            f"-> cámara=({cx:.2f},{cy:.2f},{cz:.2f})",
            throttle_duration_sec=1.0
        )

        state = (round(cx, 2), round(cy, 2), round(cz, 2))
        if state == self._last_state:
            return
        self._last_state = state

        req = SetEntityState.Request()
        req.state.name = self.entity_name
        req.state.pose.position.x = cx
        req.state.pose.position.y = cy
        req.state.pose.position.z = cz
        # Pitch de 90 grados (mirando hacia abajo), igual que el spawn original.
        req.state.pose.orientation.x = 0.0
        req.state.pose.orientation.y = 0.70710678
        req.state.pose.orientation.z = 0.0
        req.state.pose.orientation.w = 0.70710678
        req.state.reference_frame = 'world'

        future = self.cli.call_async(req)
        future.add_done_callback(self._on_response)

    def _on_response(self, future):
        try:
            res = future.result()
            if not res.success:
                self.get_logger().warn("Camera Zoom — SetEntityState devolvió success=False")
        except Exception as e:
            self.get_logger().error(f"Camera Zoom — falló la llamada al servicio: {e!r}")


def main(args=None):

    rclpy.init(args=args)
    node = CameraZoom()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
