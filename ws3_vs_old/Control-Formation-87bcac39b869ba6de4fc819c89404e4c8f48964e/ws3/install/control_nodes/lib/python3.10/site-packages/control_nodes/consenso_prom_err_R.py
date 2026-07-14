import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from multi_robot_interfaces.msg import RobotState

from control_nodes.algorithms.consensus_prom_err import ConsensusPromErr

import math


class ConsensoPromErrR(Node):

    def __init__(self):
        super().__init__('consenso_prom_err_R')

        self.robot_id = self.get_namespace().strip('/')

        self.declare_parameter('neighbor_radius', 3.0)
        self.R = self.get_parameter('neighbor_radius').value

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.states = {}
        self.algorithm = ConsensusPromErr()

        # Subscripciones
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.create_subscription(RobotState, '/robot_states', self.state_callback, 10)

        # Publicadores
        self.state_pub = self.create_publisher(RobotState, '/robot_states', 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info(
            f"{self.robot_id} consenso con R={self.R} iniciado"
        )

    # -----------------------------------------------------

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w*q.z + q.x*q.y)
        cosy_cosp = 1 - 2*(q.y*q.y + q.z*q.z)
        self.theta = math.atan2(siny_cosp, cosy_cosp)

        state = RobotState()
        state.robot_id = self.robot_id
        state.x = self.x
        state.y = self.y
        state.theta = self.theta

        self.state_pub.publish(state)

    # -----------------------------------------------------

    def state_callback(self, msg):
        self.states[msg.robot_id] = (msg.x, msg.y, msg.theta)

    # -----------------------------------------------------

    def control_loop(self):

        neighbors = []

        for rid, (xj, yj, _) in self.states.items():
            if rid == self.robot_id:
                continue

            dx = xj - self.x
            dy = yj - self.y
            dist = math.sqrt(dx*dx + dy*dy)

            if dist < self.R:
                neighbors.append((xj, yj))

        v, w = self.algorithm.compute(
            (self.x, self.y, self.theta),
            neighbors
        )

        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w
        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ConsensoPromErrR()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()