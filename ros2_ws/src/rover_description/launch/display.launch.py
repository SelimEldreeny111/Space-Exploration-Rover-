"""Display the MISSION X rover model in RViz.

    ros2 launch rover_description display.launch.py
    ros2 launch rover_description display.launch.py gui:=false
    ros2 launch rover_description display.launch.py xacro_args:="track:=0.60 lidar_height:=0.64"
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('rover_description')
    model = LaunchConfiguration('model')
    xacro_args = LaunchConfiguration('xacro_args')
    gui = LaunchConfiguration('gui')
    rviz_config = LaunchConfiguration('rviz_config')

    robot_description = ParameterValue(
        Command(['xacro ', model, ' ', xacro_args]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument(
            'model', default_value=PathJoinSubstitution([pkg, 'urdf', 'rover.urdf.xacro']),
            description='Absolute path to the rover xacro file'),
        DeclareLaunchArgument(
            'xacro_args', default_value='',
            description='Extra xacro arguments, e.g. "track:=0.60 wheel_radius:=0.1"'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Use joint_state_publisher_gui (sliders)'),
        DeclareLaunchArgument(
            'rviz_config', default_value=PathJoinSubstitution([pkg, 'rviz', 'rover.rviz']),
            description='RViz configuration file'),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher', output='screen',
             parameters=[{'robot_description': robot_description}]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             name='joint_state_publisher_gui', condition=IfCondition(gui)),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             name='joint_state_publisher', condition=UnlessCondition(gui)),
        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
             arguments=['-d', rviz_config]),
    ])
