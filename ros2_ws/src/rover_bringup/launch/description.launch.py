"""robot_state_publisher + joint_state_publisher for the rover model.

joint_state_publisher merges the wheel angles published on
/wheel/joint_states (mobility_bridge or rover_sim) and fills the passive
rocker / bogie joints (no encoders) with zero, so the full TF tree exists.
Included by robot.launch.py and sim.launch.py.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    model = PathJoinSubstitution([FindPackageShare('rover_description'), 'urdf', 'rover.urdf.xacro'])
    robot_description = ParameterValue(
        Command(['xacro ', model, ' ', LaunchConfiguration('xacro_args')]), value_type=str)
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('xacro_args', default_value='',
                              description='Extra xacro arguments, e.g. "track:=0.60"'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher', output='screen',
             parameters=[{'robot_description': robot_description,
                          'use_sim_time': use_sim_time}]),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             name='joint_state_publisher', output='screen',
             parameters=[{'source_list': ['wheel/joint_states'], 'rate': 20,
                          'use_sim_time': use_sim_time}]),
    ])
