from setuptools import find_packages, setup

package_name = 'control_nodes'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='calafaker',
    maintainer_email='leonardo.cardenas.martinez02@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'navigate_individual_pva = control_nodes.navigate_individual_pva:main',
            'aire = control_nodes.aire:main',
            'navigate_waypoints_pva = control_nodes.navigate_waypoints_pva:main',
            'consenso_node = control_nodes.consenso_node:main',
            'trayectoria_vs_node = control_nodes.trayectoria_vs_node:main',
            'pivot_giro_node = control_nodes.pivot_giro_node:main',
            'pivot_giro_waypoints_node = control_nodes.pivot_giro_waypoints_node:main',
            'pivot_giro_waypoints_v2_node = control_nodes.pivot_giro_waypoints_v2_node:main',
        ],
    },
)