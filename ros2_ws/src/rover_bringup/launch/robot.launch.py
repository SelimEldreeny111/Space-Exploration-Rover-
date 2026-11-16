"""Real-hardware bringup on the Raspberry Pi 4 (Ubuntu 24.04 + ROS 2 Jazzy).

    ros2 launch rover_bringup robot.launch.py
    ros2 launch rover_bringup robot.launch.py slam:=true            # mapping
    ros2 launch rover_bringup robot.launch.py nav:=true map:=$HOME/maps/lab.yaml
    ros2 launch rover_bringup robot.launch.py slam:=true nav:=true explore:=true

Starts robot_state_publisher (+ joint_state_publisher), the LDS-01 driver
(hls_lfcd_lds_driver), the mobility / payload / arm SSP bridges, the
ground-station (HC-12) bridge, the robot_localization EKF
(/wheel/odom + /imu/data -> /odometry/filtered, TF odom -> base_footprint)
and optionally a USB camera, slam_toolbox, Nav2 and the frontier explorer.
Device names come from config/99-missionx-rover.rules.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, NotSubstitution, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup = FindPackageShare('rover_bringup')
    hw_params = LaunchConfiguration('hardware_params')
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam = LaunchConfiguration('slam')

    def bridge(executable, port_arg):
        return Node(package='rover_hardware', executable=executable, name=executable,
                    output='screen', respawn=True, respawn_delay=2.0,
                    parameters=[hw_params, {'port': LaunchConfiguration(port_arg)}])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('hardware_params', default_value=PathJoinSubstitution(
            [FindPackageShare('rover_hardware'), 'config', 'hardware.yaml'])),
        DeclareLaunchArgument('ekf_params', default_value=PathJoinSubstitution(
            [bringup, 'config', 'ekf.yaml'])),
        DeclareLaunchArgument('mobility_port', default_value='/dev/rover_mobility'),
        DeclareLaunchArgument('payload_port', default_value='/dev/rover_payload'),
        DeclareLaunchArgument('arm_port', default_value='/dev/rover_arm'),
        DeclareLaunchArgument('radio_port', default_value='/dev/rover_radio'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/rover_lidar'),
        DeclareLaunchArgument('lidar', default_value='true', description='Start the LDS-01 driver'),
        DeclareLaunchArgument('ground_station', default_value='true',
                              description='Start the HC-12 ground-station bridge'),
        DeclareLaunchArgument('camera', default_value='false',
                              description='Start v4l2_camera -> /camera/image_raw'),
        DeclareLaunchArgument('video_device', default_value='/dev/video0'),
        DeclareLaunchArgument('slam', default_value='false'),
        DeclareLaunchArgument('nav', default_value='false'),
        DeclareLaunchArgument('explore', default_value='false'),
        DeclareLaunchArgument('map', default_value=PathJoinSubstitution(
            [FindPackageShare('rover_navigation'), 'maps', 'lab.yaml'])),
        DeclareLaunchArgument('xacro_args', default_value=''),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([bringup, 'launch', 'description.launch.py'])),
            launch_arguments={'use_sim_time': use_sim_time,
                              'xacro_args': LaunchConfiguration('xacro_args')}.items()),

        Node(package='hls_lfcd_lds_driver', executable='hlds_laser_publisher',
             name='hlds_laser_publisher', output='screen', respawn=True, respawn_delay=2.0,
             condition=IfCondition(LaunchConfiguration('lidar')),
             parameters=[{'port': LaunchConfiguration('lidar_port'), 'frame_id': 'laser_link'}]),

        bridge('mobility_bridge', 'mobility_port'),
        bridge('payload_bridge', 'payload_port'),
        bridge('arm_bridge', 'arm_port'),
        Node(package='rover_hardware', executable='ground_station_bridge',
             name='ground_station_bridge', output='screen', respawn=True, respawn_delay=2.0,
             condition=IfCondition(LaunchConfiguration('ground_station')),
             parameters=[hw_params, {'port': LaunchConfiguration('radio_port')}]),

        Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node',
             output='screen',
             parameters=[LaunchConfiguration('ekf_params'), {'use_sim_time': use_sim_time}]),

        Node(package='v4l2_camera', executable='v4l2_camera_node', name='camera',
             output='screen', condition=IfCondition(LaunchConfiguration('camera')),
             parameters=[{'video_device': LaunchConfiguration('video_device'),
                          'image_size': [640, 480], 'camera_frame_id': 'camera_optical_link'}],
             remappings=[('image_raw', 'camera/image_raw'),
                         ('camera_info', 'camera/camera_info')]),

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
                         {'use_sim_time': use_sim_time}]),
    ])
