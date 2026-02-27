import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from control_nodes.algorithms.control_law import ControlLaw

import math


class NavigateIndividualPVA(Node):

    def __init__(self):
        super().__init__('navigate_individual_pva')

        # Parámetros de objetivo
        self.declare_parameter('goal_x', 0.0)
        self.declare_parameter('goal_y', 0.0)

        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value

        self.tolerance = 0.05

        # Control law base (sin restricciones aún)
        self.controller = ControlLaw()

        # Aquí después irá el módulo PVA
        # self.pva = PVA()

        # Estado actual
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Subscripción a odom
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )

        # Publicador de cmd_vel
        self.cmd_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        # Timer de control
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info(
            f"[PVA] Navegando hacia ({self.goal_x}, {self.goal_y})"
        )

    # --------------------------------------------------

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # Extraer orientación yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.theta = math.atan2(siny_cosp, cosy_cosp)

        # Actualizar estado interno del controlador
        self.controller.x = self.x
        self.controller.y = self.y
        self.controller.theta = self.theta

    # --------------------------------------------------

    def control_loop(self):

        distance = math.sqrt(
            (self.goal_x - self.x) ** 2 +
            (self.goal_y - self.y) ** 2
        )

        if distance < self.tolerance:

            self.get_logger().info("[PVA] Objetivo alcanzado. Cerrando nodo.")

            stop = Twist()
            self.cmd_pub.publish(stop)

            self.destroy_node()
            rclpy.shutdown()
            return

        # 1️⃣ Ley de control nominal
        v_nom, w_nom = self.controller.ley_control(
            self.goal_x,
            self.goal_y
        )

        # 2️⃣ Aquí después aplicarás el PVA
        # v, w = self.pva.filtrar(v_nom, w_nom)
        v = v_nom
        w = w_nom

        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w

        self.cmd_pub.publish(twist)


# --------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = NavigateIndividualPVA()
    rclpy.spin(node)