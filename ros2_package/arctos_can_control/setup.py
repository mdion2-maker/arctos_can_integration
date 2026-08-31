from setuptools import find_packages, setup

package_name = 'arctos_can_control'

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
    maintainer='maggied',
    maintainer_email='maggied@todo.todo',
    description='ROS 2 CAN bus control node for Arctos robotic arm joints',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_control = arctos_can_control.joint_control_node:main',
            'joint_state_publisher = arctos_can_control.joint_state_publisher_node:main',
        ],
    },
)
