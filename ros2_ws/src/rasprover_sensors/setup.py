from glob import glob
from setuptools import setup

package_name = 'rasprover_sensors'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jackie167',
    maintainer_email='thi.ledinh@gmail.com',
    description='Blueprint-aligned sensor package for sensor bridges and future LiDAR/camera bringup.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'slam_sensor_bridge_node = rasprover_sensors.slam_sensor_bridge_node:main',
        ],
    },
)
