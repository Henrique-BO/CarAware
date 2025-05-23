from setuptools import setup
import os
from glob import glob

package_name = 'caraware_ros'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name), ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Henrique Barros Oliveira',
    maintainer_email='henrique.barrosoliveira@usp.br',
    description='ROS2 package for CarAware',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'model_bridge = caraware_ros.model_bridge:main',
            'vesc_republisher = caraware_ros.vesc_republisher:main',
            'carla_republisher = caraware_ros.carla_republisher:main',
            'goal_controller = caraware_ros.goal_controller:main',
            'stanley_controller = caraware_ros.stanley_controller:main',
            'square_planner = caraware_ros.square_planner:main',
        ],
    },
)
