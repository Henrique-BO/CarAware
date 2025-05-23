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
    description='Combined launch file for CARLA ROS bridge and model node in sim-test mode.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'model_bridge = caraware_ros.model_bridge:main',
            'vesc_republisher = caraware_ros.vesc_republisher:main',
            'carla_republisher = caraware_ros.carla_republisher:main',
            'controller = caraware_ros.controller:main',
        ],
    },
)
