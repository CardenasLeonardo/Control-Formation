import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity


# ------------------------------------------------------------------
# Demo didáctico: spawnea UN cubo con propiedades físicas completas
# (masa, inercia, colisión) — a diferencia de las esferas de referencia
# de la estructura virtual (kinematic=true, sin colisión), este cubo
# es un cuerpo rígido normal: cae por gravedad y puede chocar.
# Sirve como caso base para explicar qué campos definen un modelo SDF
# antes de mostrar la variante "sin física" usada en el resto del
# proyecto (ver Sections/Despliegue de estructura virtual/).
# ------------------------------------------------------------------

SIZE  = 0.4                     # lado del cubo, en metros
MASS  = 1.0                     # kg
COLOR = '0.85 0.3 0.1 1.0'      # RGBA


def _cube_inertia(mass, size):
    # Cubo sólido homogéneo: I = (1/6) * m * a^2 en cada eje.
    i = (mass * size ** 2) / 6.0
    return i, i, i


def _make_sdf_cube(name, size, mass, color):
    ixx, iyy, izz = _cube_inertia(mass, size)
    return f"""<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{name}">
    <static>false</static>
    <link name="link">
      <inertial>
        <mass>{mass}</mass>
        <inertia>
          <ixx>{ixx}</ixx><iyy>{iyy}</iyy><izz>{izz}</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry><box><size>{size} {size} {size}</size></box></geometry>
      </collision>
      <visual name="visual">
        <geometry><box><size>{size} {size} {size}</size></box></geometry>
        <material>
          <ambient>{color}</ambient>
          <diffuse>{color}</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""


class SpawnCubeDemoNode(Node):

    def __init__(self):
        super().__init__('spawn_cube_demo')

        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 1.0)   # cae desde 1 m de altura

        self.x = self.get_parameter('x').value
        self.y = self.get_parameter('y').value
        self.z = self.get_parameter('z').value

        self.spawn_cli = self.create_client(SpawnEntity, '/spawn_entity')
        self._spawned = False
        self.create_timer(0.1, self.tick)

    def tick(self):
        if self._spawned:
            return
        if not self.spawn_cli.service_is_ready():
            return

        req = SpawnEntity.Request()
        req.name                       = 'cubo_demo'
        req.xml                        = _make_sdf_cube('cubo_demo', SIZE, MASS, COLOR)
        req.initial_pose.position.x    = self.x
        req.initial_pose.position.y    = self.y
        req.initial_pose.position.z    = self.z
        req.initial_pose.orientation.w = 1.0
        req.reference_frame            = 'world'
        self.spawn_cli.call_async(req)
        self._spawned = True
        self.get_logger().info(f'Cubo demo spawneado en ({self.x}, {self.y}, {self.z})')


def main(args=None):
    rclpy.init(args=args)
    node = SpawnCubeDemoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
