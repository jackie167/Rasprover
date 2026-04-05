from setuptools import setup

package_name = 'rasprover_web'

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
    description='Legacy compatibility wrapper for the Rasprover UI bridge.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'web_bridge_node = rasprover_web.web_bridge_node:main',
        ],
    },
)
