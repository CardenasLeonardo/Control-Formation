import rclpy
from rclpy.node import Node

from multi_robot_interfaces.msg import RobotState

import matplotlib.pyplot as plt
import time


class StatesPlotter(Node):

    def __init__(self):

        super().__init__('states_plotter')

        self.sub = self.create_subscription(
            RobotState,
            '/robot_states_rx',
            self.state_callback,
            10
        )

        # almacenamiento
        self.data = {}

        self.start_time = time.time()

        plt.style.use('ggplot')

        plt.ion()

        self.fig, self.ax = plt.subplots(1,3, figsize=(15,5))

        plt.show(block=False)

        # TIMER PARA ACTUALIZAR EL PLOT
        self.timer = self.create_timer(0.1, self.update_plot)

        self.get_logger().info("States Plotter iniciado")

    def state_callback(self, msg):

        robot = msg.robot_id
        x = msg.x
        y = msg.y

        t = time.time() - self.start_time

        if robot not in self.data:

            self.data[robot] = {
                "t": [],
                "x": [],
                "y": []
            }

        self.data[robot]["t"].append(t)
        self.data[robot]["x"].append(x)
        self.data[robot]["y"].append(y)

    def update_plot(self):

        # limpiar ejes
        for a in self.ax:
            a.cla()

        # ---------- TITULOS ----------
        self.ax[0].set_title("Trayectoria de los robots")
        self.ax[1].set_title("Evolución de x en el tiempo")
        self.ax[2].set_title("Evolución de y en el tiempo")

        # ---------- EJES ----------
        self.ax[0].set_xlabel("x [m]")
        self.ax[0].set_ylabel("y [m]")

        self.ax[1].set_xlabel("tiempo [s]")
        self.ax[1].set_ylabel("x [m]")

        self.ax[2].set_xlabel("tiempo [s]")
        self.ax[2].set_ylabel("y [m]")

        # ---------- GRID ----------
        for a in self.ax:
            a.grid(True)

        # ---------- PLOTS ----------
        for robot, d in self.data.items():

            self.ax[0].plot(d["x"], d["y"], label=robot)
            self.ax[1].plot(d["t"], d["x"], label=robot)
            self.ax[2].plot(d["t"], d["y"], label=robot)

        # ---------- LEYENDAS ----------
        for a in self.ax:
            if self.data:
                a.legend()

        # ---------- ESCALA IGUAL PARA XY ----------
        self.ax[0].set_aspect('equal', adjustable='box')

        # ---------- ACTUALIZAR FIGURA ----------
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

def main(args=None):

    rclpy.init(args=args)

    node = StatesPlotter()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()