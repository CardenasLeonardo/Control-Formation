from setuptools import setup
from setuptools import find_packages
import os
from glob import glob

package_name = 'simulation_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        # Para que ROS2 detecte el paquete
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        # Instala package.xml
        ('share/' + package_name, ['package.xml']),

        # INSTALA LOS LAUNCH FILES
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='calafaker',
    maintainer_email='example@example.com',
    description='HMI y simulación multi-robot',
    license='Apache License 2.0',
    entry_points={
    'console_scripts': [
        'hmi_gui = simulation_pkg.hmi_gui:main',
        'hmi_node = simulation_pkg.hmi_visual:main',  # luego lo haremos
        'hmi_visual = simulation_pkg.hmi_visual:main',
        ],
    },

)
