import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from multi_robot_interfaces.msg import RobotState

import math


class DistributedController(Node):

    def __init__(self):
        super().__init__('distributed_controller')

        # Obtener namespace real del robot
        self.robot_id = self.get_namespace().strip('/')

        # Parámetro radio R
        self.declare_parameter('neighbor_radius', 2.0)
        self.R = self.get_parameter('neighbor_radius').value

        # Estado propio
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Diccionario de vecinos
        self.neighbors = {}

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )

        self.state_sub = self.create_subscription(
            RobotState,
            '/robot_states',
            self.state_callback,
            10
        )

        # Publishers
        self.state_pub = self.create_publisher(
            RobotState,
            '/robot_states',
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        # Control loop timer
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info(f"{self.robot_id} controller started")

    # --------------------------------------------------

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # Publicar estado propio
        state_msg = RobotState()
        state_msg.robot_id = self.robot_id
        state_msg.x = self.x
        state_msg.y = self.y
        state_msg.theta = 0.0  # simplificado

        self.state_pub.publish(state_msg)

    # --------------------------------------------------

    def state_callback(self, msg):

        # Ignorar mi propio estado
        if msg.robot_id == self.robot_id:
            return

        dx = msg.x - self.x
        dy = msg.y - self.y
        dist = math.sqrt(dx*dx + dy*dy)

        if dist < self.R:
            self.neighbors[msg.robot_id] = (msg.x, msg.y)
        else:
            if msg.robot_id in self.neighbors:
                del self.neighbors[msg.robot_id]

    # --------------------------------------------------

    def control_loop(self):

        twist = Twist()

        if len(self.neighbors) > 0:

            # Consenso simple hacia promedio de vecinos
            avg_x = 0.0
            avg_y = 0.0

            for (nx, ny) in self.neighbors.values():
                avg_x += nx
                avg_y += ny

            avg_x /= len(self.neighbors)
            avg_y /= len(self.neighbors)

            error_x = avg_x - self.x
            error_y = avg_y - self.y

            twist.linear.x = 0.5 * error_x
            twist.angular.z = 0.0

        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self.cmd_pub.publish(twist)


# --------------------------------------------------


def main(args=None):
    rclpy.init(args=args)
    node = DistributedController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()