"""Online asynchronous SLAM with slam_toolbox (replaces ROS 1 hector_slam).

    ros2 launch rover_slam slam.launch.py                       # real robot
    ros2 launch rover_slam slam.launch.py use_sim_time:=true    # with rover_sim

Requires /scan and TF odom -> base_footprint (EKF). Publishes /map and
TF map -> odom. Save the map with
    ros2 run nav2_map_server map_saver_cli -f ~/maps/lab

slam_toolbox's own online_async_launch.py is included so that the lifecycle
handling of the installed slam_toolbox version (Jazzy: lifecycle node with
autostart) is used unchanged.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    params = LaunchConfiguration('slam_params_file')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use /clock (simulation)'),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('rover_slam'), 'config', 'slam_toolbox_online_async.yaml']),
            description='slam_toolbox parameter file'),
        DeclareLaunchArgument('autostart', default_value='true',
                              description='Configure + activate slam_toolbox automatically'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py'])),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'slam_params_file': params,
                'autostart': LaunchConfiguration('autostart'),
            }.items()),
    ])
