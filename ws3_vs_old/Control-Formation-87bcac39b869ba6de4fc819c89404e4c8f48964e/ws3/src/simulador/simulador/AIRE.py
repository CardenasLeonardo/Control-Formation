import math

import rclpy
from rclpy.node import Node

from multi_robot_interfaces.msg import RobotState


class Aire(Node):

    def __init__(self):

        super().__init__('aire')

        self.declare_parameter('neighbor_radius', 3.0)
        self.R = float(self.get_parameter('neighbor_radius').value)
        self.states = {}              # { robot_id: (x, y, theta) }
        self._neighbor_pubs = {}      # { robot_id: Publisher }

        self.sub = self.create_subscription(
            RobotState,
            '/robot_states_tx',
            self.state_callback,
            10
        )

        self.get_logger().info(f"Nodo AIRE iniciado con R={self.R} m")

    # --------------------------------------------------

    def get_publisher(self, robot_id):
        """Crea el publisher para un robot si no existe todavía."""

        if robot_id not in self._neighbor_pubs:
            topic = f'/{robot_id}/neighbors_rx'
            
            self._neighbor_pubs[robot_id] = self.create_publisher(
                RobotState,
                topic,
                10
            )

            self.get_logger().info(f"Publisher creado: {topic}")

        return self._neighbor_pubs[robot_id]

    # --------------------------------------------------

    def state_callback(self, msg):

        # Actualizar estado del robot emisor
        self.states[msg.robot_id] = (msg.x, msg.y, msg.theta)

        # Asegurar publisher para este robot
        self.get_publisher(msg.robot_id)

        # Para cada robot conocido, recalcular y publicar sus vecinos
        for robot_id, (xi, yi, ti) in self.states.items():

            pub = self.get_publisher(robot_id)

            for neighbor_id, (xj, yj, tj) in self.states.items():

                if neighbor_id == robot_id:
                    continue

                dist = math.sqrt((xj - xi)**2 + (yj - yi)**2)

                if dist <= self.R:

                    neighbor_msg = RobotState()
                    neighbor_msg.robot_id = neighbor_id
                    neighbor_msg.x = xj
                    neighbor_msg.y = yj
                    neighbor_msg.theta = tj

                    pub.publish(neighbor_msg)


def main(args=None):

    rclpy.init(args=args)

    node = Aire()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()