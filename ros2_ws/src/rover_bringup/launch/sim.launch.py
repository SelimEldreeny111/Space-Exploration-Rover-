"""Simulation bringup: rover_sim replaces the MCUs and the LiDAR.

    ros2 launch rover_bringup sim.launch.py                      # sim + EKF + RViz
    ros2 launch rover_bringup sim.launch.py slam:=true           # mapping (drive with teleop)
    ros2 launch rover_bringup sim.launch.py nav:=true            # AMCL on maps/lab.yaml + Nav2
    ros2 launch rover_bringup sim.launch.py slam:=true nav:=true explore:=true explore_autostart:=true

Everything except rover_sim runs on simulated time (/clock from rover_sim).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, NotSubstitution, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup = FindPackageShare('rover_bringup')
    slam = LaunchConfiguration('slam')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Stamp with and follow the simulated /clock'),
        DeclareLaunchArgument('world', default_value=PathJoinSubstitution(
            [FindPackageShare('rover_sim'), 'worlds', 'lab.yaml']),
            description='rover_core.sim world file ("" = World.default_lab())'),
        DeclareLaunchArgument('real_time_factor', default_value='1.0'),
        DeclareLaunchArgument('seed', default_value='0'),
        DeclareLaunchArgument('slam', default_value='false'),
        DeclareLaunchArgument('nav', default_value='false'),
        DeclareLaunchArgument('explore', default_value='false'),
        DeclareLaunchArgument('explore_autostart', default_value='false',
                              description='Explorer starts without /rover/mode == AUTONOMOUS'),
        DeclareLaunchArgument('map', default_value=PathJoinSubstitution(
            [FindPackageShare('rover_navigation'), 'maps', 'lab.yaml']),
            description='Map for AMCL when nav:=true and slam:=false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('ekf_params', default_value=PathJoinSubstitution(
            [bringup, 'config', 'ekf.yaml'])),

        # explicit types: world:="" must stay an (empty) string and
        # real_time_factor:=2 a double (plain substitutions are YAML-typed)
        Node(package='rover_sim', executable='rover_sim_node', name='rover_sim', output='screen',
             parameters=[PathJoinSubstitution([FindPackageShare('rover_sim'), 'config', 'sim.yaml']),
                         {'world': ParameterValue(LaunchConfiguration('world'), value_type=str),
                          'real_time_factor': ParameterValue(
                              LaunchConfiguration('real_time_factor'), value_type=float),
                          'seed': LaunchConfiguration('seed'),
                          'publish_clock': use_sim_time,
                          'use_sim_time': False}]),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([bringup, 'launch', 'description.launch.py'])),
            launch_arguments={'use_sim_time': use_sim_time}.items()),

        Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node',
             output='screen',
             parameters=[LaunchConfiguration('ekf_params'), {'use_sim_time': use_sim_time}]),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('rover_slam'), 'launch', 'slam.launch.py'])),
            condition=IfCondition(slam),
            launch_arguments={'use_sim_time': use_sim_time}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('rover_navigation'), 'launch', 'navigation.launch.py'])),
            condition=IfCondition(LaunchConfiguration('nav')),
            launch_arguments={'use_sim_time': use_sim_time,
                              'localization': NotSubstitution(slam),
                              'map': LaunchConfiguration('map')}.items()),
        Node(package='rover_mission', executable='explorer', name='explorer', output='screen',
             condition=IfCondition(LaunchConfiguration('explore')),
             parameters=[PathJoinSubstitution([FindPackageShare('rover_mission'), 'config',
                                               'explorer.yaml']),
                         {'use_sim_time': use_sim_time,
                          'autostart': LaunchConfiguration('explore_autostart')}]),
        Node(package='rviz2', executable='rviz2', name='rviz2', output='log',
             condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', PathJoinSubstitution([bringup, 'rviz', 'rover_nav.rviz'])],
             parameters=[{'use_sim_time': use_sim_time}]),
    ])
