import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
from tf_transformations import euler_from_quaternion

from control_algorithms.algorithms.control_law import ControlLaw
#from control_algorithms.algorithms.pva import PVAReal


class MultiController(Node):

    def __init__(self, robot_names):
        super().__init__('multi_robot_controller')

        self.robot_names = robot_names                                              # Lista de robots a controlar 
        self.targets = {                                                            # Metas entre las que se moverá cada robot (A <-> B)
            "robot0": [(2.0, 2.0), (8.0, 6.0)],
            "robot1": [(-2.0, 2.0), (-8.0, 6.0)],
            "robot2": [(-2.0, 1.0), (-8.0, -6.0)]
        }

        self.current_target = {name: 0 for name in robot_names}                     # Índice de meta actual (0 o 1)
        self.states = {name: None for name in robot_names}                          # Estado (x, y, theta)
        self.latest_scan = {name: None for name in robot_names}                     # Última lectura del LIDAR

        # Controladores individuales
        self.controllers = {name: ControlLaw() for name in robot_names}

    

        # --- Suscripciones a ODOM ---
        for name in robot_names:
            self.create_subscription(
                Odometry,
                f"/{name}/odom",
                lambda msg, n=name: self.odom_callback(msg, n),
                10
            )

        # --- Suscripciones a SCAN ---
        for name in robot_names:
            self.create_subscription(
                LaserScan,
                f"/{name}/scan",
                lambda msg, n=name: self.scan_callback(msg, n),
                10
            )

        # --- Publishers para los cmd_vel ---
        self.cmd_vel_publishers = {
            name: self.create_publisher(Twist, f"/{name}/cmd_vel", 10)
            for name in robot_names
        }

        # Timer principal (20 Hz)
        self.timer = self.create_timer(0.05, self.control_loop)

    # ---------------------------------------------------------
    # CALLBACKS
    # ---------------------------------------------------------

    def odom_callback(self, msg, robot_name):
        pose = msg.pose.pose

        # Convertir quaternion -> yaw real
        q = pose.orientation
        quaternion = (q.x, q.y, q.z, q.w)
        (_, _, yaw) = euler_from_quaternion(quaternion)

        self.states[robot_name] = (
            pose.position.x,
            pose.position.y,
            yaw
        )

    def scan_callback(self, msg, robot_name):
        """Almacena la última lectura del SCAN para ese robot."""
        self.latest_scan[robot_name] = msg

    # ---------------------------------------------------------
    # CONTROL LOOP PRINCIPAL
    # ---------------------------------------------------------

    def control_loop(self):

        for name in self.robot_names:

            if self.states[name] is None:
                continue

            x, y, theta = self.states[name]

            # Actualizar el controlador
            ctrl = self.controllers[name]
            ctrl.x = x
            ctrl.y = y
            ctrl.theta = theta

            # Obtener meta actual
            goals = self.targets[name]
            idx = self.current_target[name]
            xr, yr = goals[idx]

            # Ley de control exponencial (sin PVA aún)
            v, w = ctrl.ley_control(xr, yr)

        
            # Ver si llegó a la meta
            dist = math.sqrt((xr - x)**2 + (yr - y)**2)
            if dist < 0.25:
                self.current_target[name] = 1 - idx  # alternar entre 0 <-> 1

            # Publicar velocidades finales
            msg = Twist()
            msg.linear.x = v
            msg.angular.z = w
            self.cmd_vel_publishers[name].publish(msg)


def main(args=None):
    rclpy.init(args=args)

    robot_names = ["robot0", "robot1", "robot2"]

    node = MultiController(robot_names)
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
