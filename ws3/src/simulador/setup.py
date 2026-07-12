from setuptools import setup
from glob import glob
import os

package_name = 'simulador'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),

        (os.path.join('share', package_name, 'models'),
            glob('models/*.sdf')),

        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='calafaker',
    description='Simulador multi-robot',
    license='Apache License 2.0',
    tests_require=['pytest'],

    entry_points={
        'console_scripts': [
            'aire              = simulador.AIRE:main',
            'states_plotter_v2 = simulador.states_plotter_v2:main',
            'pva_plotter        = simulador.pva_plotter:main',
            'sphere_demo        = simulador.sphere_demo:main',
            'vs_node            = simulador.vs_node:main',
            'gif_recorder       = simulador.gif_recorder:main',
            'camera_zoom        = simulador.camera_zoom:main',
        ],
    },
)