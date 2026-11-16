"""Nav2 (Jazzy) bringup for the MISSION X rover.

    # localization on a saved map (map_server + AMCL) + navigation
    ros2 launch rover_navigation navigation.launch.py map:=/path/to/map.yaml
    # navigation while slam_toolbox provides /map and map -> odom (exploration)
    ros2 launch rover_navigation navigation.launch.py localization:=false
    # simulation
    ros2 launch rover_navigation navigation.launch.py use_sim_time:=true

The node set mirrors nav2_bringup's navigation_launch.py + localization_launch.py
(Jazzy) without collision_monitor / docking_server, so the parameter file
contains everything that is started:
  controller_server -> cmd_vel_nav -> velocity_smoother -> /cmd_vel
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

NAV_NODES = ['controller_server', 'smoother_server', 'planner_server', 'behavior_server',
             'velocity_smoother', 'bt_navigator', 'waypoint_follower']
LOC_NODES = ['map_server', 'amcl']


def generate_launch_description():
    pkg = FindPackageShare('rover_navigation')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    localization = LaunchConfiguration('localization')
    log_level = LaunchConfiguration('log_level')
    common = [params, {'use_sim_time': use_sim_time}]
    ros_args = ['--ros-args', '--log-level', log_level]

    def nav2_node(package, executable, remappings=(), extra_params=None):
        return Node(package=package, executable=executable, name=executable, output='screen',
                    parameters=common + ([extra_params] if extra_params else []),
                    remappings=list(remappings), arguments=ros_args,
                    respawn=False)

    localization_group = GroupAction(
        condition=IfCondition(localization),
        actions=[
            nav2_node('nav2_map_server', 'map_server',
                      extra_params={'yaml_filename': LaunchConfiguration('map')}),
            nav2_node('nav2_amcl', 'amcl'),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                 name='lifecycle_manager_localization', output='screen', arguments=ros_args,
                 parameters=[{'use_sim_time': use_sim_time, 'autostart': autostart,
                              'node_names': LOC_NODES}]),
        ])

    navigation_group = GroupAction(actions=[
        nav2_node('nav2_controller', 'controller_server', [('cmd_vel', 'cmd_vel_nav')]),
        nav2_node('nav2_smoother', 'smoother_server'),
        nav2_node('nav2_planner', 'planner_server'),
        nav2_node('nav2_behaviors', 'behavior_server', [('cmd_vel', 'cmd_vel_nav')]),
        nav2_node('nav2_bt_navigator', 'bt_navigator'),
        nav2_node('nav2_waypoint_follower', 'waypoint_follower'),
        nav2_node('nav2_velocity_smoother', 'velocity_smoother',
                  [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen', arguments=ros_args,
             parameters=[{'use_sim_time': use_sim_time, 'autostart': autostart,
                          'node_names': NAV_NODES}]),
    ])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('params_file',
                              default_value=PathJoinSubstitution([pkg, 'config', 'nav2_params.yaml']),
                              description='Nav2 parameter file'),
        DeclareLaunchArgument('map', default_value=PathJoinSubstitution([pkg, 'maps', 'lab.yaml']),
                              description='Map for map_server (localization:=true)'),
        DeclareLaunchArgument('localization', default_value='true',
                              description='Start map_server + AMCL (false when SLAM runs)'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('log_level', default_value='info'),
        localization_group,
        navigation_group,
    ])
