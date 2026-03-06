import rclpy
from rclpy.node import Node

from multi_robot_interfaces.msg import RobotState


class Aire(Node):

    def __init__(self):

        super().__init__('aire')

        # Subscriber (robots transmiten)
        self.sub = self.create_subscription(
            RobotState,
            '/robot_states_tx',
            self.state_callback,
            10
        )

        # Publisher (aire transmite)
        self.pub = self.create_publisher(
            RobotState,
            '/robot_states_rx',
            10
        )

        self.get_logger().info("Nodo AIRE iniciado")

    def state_callback(self, msg):

        # Por ahora solo retransmitimos
        self.pub.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    node = Aire()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()