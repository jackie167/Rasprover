from glob import glob
from setuptools import setup

package_name = 'rasprover_slam'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/config', glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jackie167',
    maintainer_email='thi.ledinh@gmail.com',
    description='Blueprint-aligned SLAM package for launches and parameters.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odometry_path_node = rasprover_slam.odometry_path_node:main',
            'navigate_to_pose_cli = rasprover_slam.navigate_to_pose_cli:main',
            'set_initial_pose_cli = rasprover_slam.set_initial_pose_cli:main',
            'frontier_explorer_node = rasprover_slam.frontier_explorer_node:main',
            'simple_odom_filter_node = rasprover_slam.simple_odom_filter_node:main',
        ],
    },
)
