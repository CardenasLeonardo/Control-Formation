import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity


# ------------------------------------------------------------------
# Demo didáctico: spawnea las piezas SUELTAS que componen el robot real
# (articubot_one), fieles a los materiales/geometrías definidos en
# robot_core.xacro y lidar.xacro, para mostrar que son componentes
# configurables por separado, no un modelo monolítico.
#   - chassis: caja blanca 0.3x0.3x0.15
#   - rueda izquierda / derecha: cilindro azul r=0.05 l=0.04
#   - rueda castor: esfera negra r=0.05
#   - LiDAR (laser_frame): cilindro rojo r=0.05 l=0.04
# ------------------------------------------------------------------

WHITE = '1.0 1.0 1.0 1.0'
BLUE  = '0.2 0.2 1.0 1.0'
BLACK = '0.0 0.0 0.0 1.0'
RED   = '1.0 0.0 0.0 1.0'


def _sdf_box(name, size, color):
    sx, sy, sz = size
    return f"""<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>"""


def _sdf_cylinder(name, radius, length, color):
    return f"""<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry><cylinder><radius>{radius}</radius><length>{length}</length></cylinder></geometry>
        <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>"""


def _sdf_sphere(name, radius, color):
    return f"""<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry><sphere><radius>{radius}</radius></sphere></geometry>
        <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>"""


class SpawnPartsDemoNode(Node):

    def __init__(self):
        super().__init__('spawn_parts_demo')

        # Fila de piezas, separadas 0.5 m entre sí, frente al robot.
        self.declare_parameter('start_x', 0.7)
        self.declare_parameter('start_y', -0.8)
        self.declare_parameter('spacing', 0.5)

        self.start_x = self.get_parameter('start_x').value
        self.start_y = self.get_parameter('start_y').value
        self.spacing = self.get_parameter('spacing').value

        self.spawn_cli = self.create_client(SpawnEntity, '/spawn_entity')
        self._parts = self._build_parts()
        self._idx = 0
        self.create_timer(0.2, self.tick)

    def _build_parts(self):
        z_box  = 0.075   # mitad de la altura del chasis (0.15/2)
        z_flat = 0.05    # radio de cilindros/esfera, asentados en el piso

        # (nombre, sdf, z, "radio" en Y — mitad del ancho de la pieza en
        # ese eje, para calcular separación borde a borde, no centro a
        # centro; el chasis es mucho más ancho que el resto).
        specs = [
            ('chassis_demo', _sdf_box('chassis_demo', (0.3, 0.3, 0.15), WHITE), z_box,  0.15),
            ('wheel_l_demo', _sdf_cylinder('wheel_l_demo', 0.05, 0.04, BLUE),   z_flat, 0.05),
            ('wheel_r_demo', _sdf_cylinder('wheel_r_demo', 0.05, 0.04, BLUE),   z_flat, 0.05),
            ('caster_demo',  _sdf_sphere('caster_demo', 0.05, BLACK),          z_flat, 0.05),
            ('lidar_demo',   _sdf_cylinder('lidar_demo', 0.05, 0.04, RED),     z_flat, 0.05),
        ]

        # Offset acumulado: borde a borde (radio actual + gap + radio
        # siguiente), no un spacing uniforme centro a centro.
        parts = []
        y_offset = 0.0
        prev_half_width = 0.0
        for i, (name, xml, z, half_width) in enumerate(specs):
            if i > 0:
                y_offset += prev_half_width + self.spacing + half_width
            parts.append((name, xml, z, y_offset))
            prev_half_width = half_width
        return parts

    def tick(self):
        if self._idx >= len(self._parts):
            return
        if not self.spawn_cli.service_is_ready():
            return

        name, xml, z, y_offset = self._parts[self._idx]
        x = self.start_x
        y = self.start_y + y_offset

        req = SpawnEntity.Request()
        req.name                       = name
        req.xml                        = xml
        req.initial_pose.position.x    = x
        req.initial_pose.position.y    = y
        req.initial_pose.position.z    = z
        req.initial_pose.orientation.w = 1.0
        req.reference_frame            = 'world'
        self.spawn_cli.call_async(req)

        self.get_logger().info(f'Spawneada pieza {name} en ({x:.2f}, {y:.2f}, {z:.2f})')
        self._idx += 1


def main(args=None):
    rclpy.init(args=args)
    node = SpawnPartsDemoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
