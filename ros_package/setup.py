"""webots_ros2 package setup file."""
from setuptools import setup
from glob import glob
package_name = 'webots_ros2_robomaster'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/robot_launch.py']))
data_files.append(('share/' + package_name + '/resource', [
    'resource/robomaster.urdf',
    'resource/ros2control.yml',
    'resource/default.rviz',
    'resource/robomaster_world_map.pgm',
    'resource/robomaster_world_map.yaml',
]))
data_files.append(('share/' + package_name + '/protos/meshes', [
    'protos/meshes/Orin.dae',
    'protos/meshes/Robomaster-车体_output.obj',
    'protos/meshes/Robomaster-车轮_output.obj',
]))
data_files.append(('share/' + package_name + '/img', [
    'img/aruco_23_640x640_.png',
    'img/aruco_1_640x640_.png',
    'img/aruco_2_640x640_.png',
    'img/aruco_3_640x640_.png',
    'img/aruco_4_640x640_.png',
    'img/aruco_1_640x640_bordered.png',
    'img/aruco_2_640x640_bordered.png',
    'img/aruco_3_640x640_bordered.png',
    'img/aruco_4_640x640_bordered.png',
]))
data_files.append(('share/' + package_name + '/worlds', glob('worlds/*')))
data_files.append(('share/' + package_name, ['package.xml']))
setup(
    name=package_name,
    version='2025.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools', 'launch'],
    zip_safe=True,
    author='Cyberbotics',
    author_email='support@cyberbotics.com',
    maintainer='Cyberbotics',
    maintainer_email='support@cyberbotics.com',
    keywords=['ROS', 'Webots', 'Robot', 'Simulation', 'Examples'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Robomaster Burger robot ROS2 interface for Webots.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'launch.frontend.launch_extension': ['launch_ros = launch_ros'],
        'console_scripts': [
            'wheels_controller = webots_ros2_robomaster.wheels_controller:main',
            'pid_controller = webots_ros2_robomaster.pid_controller:main',
            'trajectory_tracker = webots_ros2_robomaster.trajectory_tracker:main',
            'target_follower = webots_ros2_robomaster.target_follower:main',
            'ros_mcp_server = webots_ros2_robomaster.ros_mcp_server:main',
            'part5_mcp_demo_client = webots_ros2_robomaster.part5_mcp_demo_client:main',
        ],
    }
)
