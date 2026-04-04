from setuptools import setup

package_name = 'rasprover_base'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jackie167',
    maintainer_email='thi.ledinh@gmail.com',
    description='ROS 2 hardware boundary node for Rasprover.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_base_node = rasprover_base.robot_base_node:main',
        ],
    },
)
