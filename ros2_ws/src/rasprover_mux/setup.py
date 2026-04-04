from setuptools import setup

package_name = 'rasprover_mux'

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
    description='ROS 2 motion command mux node for Rasprover.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'command_mux_node = rasprover_mux.command_mux_node:main',
            'joystick_teleop_node = rasprover_mux.joystick_teleop_node:main',
            'local_joy_node = rasprover_mux.local_joy_node:main',
        ],
    },
)
