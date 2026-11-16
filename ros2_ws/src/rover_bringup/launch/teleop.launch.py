"""Keyboard teleoperation (teleop_twist_keyboard -> /cmd_vel).

teleop_twist_keyboard needs a terminal: by default it is opened in an xterm
(sudo apt install xterm). Over SSH, run it directly instead:
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.3 -p turn:=0.6

    ros2 launch rover_bringup teleop.launch.py
    ros2 launch rover_bringup teleop.launch.py terminal_prefix:="gnome-terminal --"
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('speed', default_value='0.3', description='Linear speed [m/s]'),
        DeclareLaunchArgument('turn', default_value='0.6', description='Yaw rate [rad/s]'),
        DeclareLaunchArgument('terminal_prefix', default_value='xterm -e'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='teleop_twist_keyboard', executable='teleop_twist_keyboard',
             name='teleop_twist_keyboard', output='screen',
             prefix=LaunchConfiguration('terminal_prefix'),
             parameters=[{'speed': LaunchConfiguration('speed'),
                          'turn': LaunchConfiguration('turn'),
                          'use_sim_time': LaunchConfiguration('use_sim_time')}]),
    ])
