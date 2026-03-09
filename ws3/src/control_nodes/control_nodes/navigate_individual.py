import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from multi_robot_interfaces.msg import RobotState

from control_nodes.algorithms.control_law import ControlLaw

import math


class NavigateIndividual(Node):

    def __init__(self):
        super().__init__('navigate_individual')

        # 🔹 ID del robot desde namespace
        self.robot_id = self.get_namespace().strip('/')

        # Parámetros de objetivo
        self.declare_parameter('goal_x', 0.0)
        self.declare_parameter('goal_y', 0.0)

        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value

        self.tolerance = 0.05

        # Control law
        self.controller = ControlLaw()

        # Estado actual
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )

        # Publishers
        self.cmd_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        # 🔹 Publicación del estado al nodo AIRE
        self.state_pub = self.create_publisher(
            RobotState,
            '/robot_states_tx',
            10
        )

        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info(
            f"{self.robot_id} navegando hacia ({self.goal_x}, {self.goal_y})"
        )

    # --------------------------------------------------

    def odom_callback(self, msg):

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w*q.z + q.x*q.y)
        cosy_cosp = 1 - 2*(q.y*q.y + q.z*q.z)
        self.theta = math.atan2(siny_cosp, cosy_cosp)

        # Publicar estado al AIRE
        state = RobotState()
        state.robot_id = self.robot_id
        state.x = self.x
        state.y = self.y
        state.theta = self.theta

        self.state_pub.publish(state)

        # Actualizar controlador
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

            self.get_logger().info("Objetivo alcanzado.")

            stop = Twist()
            self.cmd_pub.publish(stop)

            return

        v, w = self.controller.ley_control(
            self.goal_x,
            self.goal_y
        )

        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = NavigateIndividual()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()