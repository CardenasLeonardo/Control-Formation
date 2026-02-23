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
        'distributed_controller = control_nodes.distributed_controller:main',
        'navigate_individual = control_nodes.navigate_individual:main',
        'consenso_prom_err_notR = control_nodes.consenso_prom_err_notR:main',
    ],
},
)
