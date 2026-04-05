from glob import glob
from setuptools import setup

package_name = 'rasprover_localization'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jackie167',
    maintainer_email='thi.ledinh@gmail.com',
    description='Legacy compatibility wrapper for Rasprover localization entrypoints.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'slam_sensor_bridge_node = rasprover_localization.slam_sensor_bridge_node:main',
        ],
    },
)
