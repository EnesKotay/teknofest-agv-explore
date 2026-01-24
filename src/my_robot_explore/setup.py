from setuptools import setup

package_name = 'my_robot_explore'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='burak',
    maintainer_email='burak@todo.todo',
    description='Frontier exploration + odom tf broadcaster',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'frontier_explorer = my_robot_explore.frontier_explorer:main',
            'odom_tf_broadcaster = my_robot_explore.odom_tf_broadcaster:main',
            'stm32_serial_bridge = my_robot_explore.stm32_serial_bridge:main',
        ],
    },
)
