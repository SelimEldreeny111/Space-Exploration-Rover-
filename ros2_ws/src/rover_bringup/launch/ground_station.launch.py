"""HC-12 ground-station bridge only (e.g. when the rest of the stack runs
elsewhere, or to restart the radio link independently).

    ros2 launch rover_bringup ground_station.launch.py
    ros2 launch rover_bringup ground_station.launch.py port:=/dev/ttyUSB1 baud:=9600
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/rover_radio'),
        DeclareLaunchArgument('baud', default_value='9600'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('hardware_params', default_value=PathJoinSubstitution(
            [FindPackageShare('rover_hardware'), 'config', 'hardware.yaml'])),
        Node(package='rover_hardware', executable='ground_station_bridge',
             name='ground_station_bridge', output='screen', respawn=True, respawn_delay=2.0,
             parameters=[LaunchConfiguration('hardware_params'),
                         {'port': LaunchConfiguration('port'),
                          'baud': LaunchConfiguration('baud'),
                          'use_sim_time': LaunchConfiguration('use_sim_time')}]),
    ])
