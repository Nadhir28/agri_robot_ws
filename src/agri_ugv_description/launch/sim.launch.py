"""Launch Gazebo Sim with the agri_ugv robot, the ROS<->Gazebo bridge and
robot_state_publisher.

Arguments:
  world:=empty_flat|olive_orchard|<path.sdf>   Default: empty_flat
  headless:=true|false   run the Gazebo server only (no GUI). Default false.
  rviz:=true|false       also start RViz (map view with slam:=true). Default false.
  camera:=true|false     simulate the front camera (rendering, ~RAM). Default true.
  lidar:=true|false      simulate the 3D LiDAR (+ derived /scan). Default true.
  slam:=true|false       build the map with slam_toolbox (olive_orchard). Default false.
  x, y, z, yaw           spawn pose. Default: the selected world's safe spawn
                         pose (see WORLDS below; 0 0 0.05 0 for a custom path).
"""
import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            ExecuteProcess, IncludeLaunchDescription, OpaqueFunction,
                            Shutdown)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PKG = get_package_share_directory('agri_ugv_description')
EKF_CONFIG = os.path.join(PKG, 'config', 'ekf.yaml')

# Named worlds and their default (collision-free) spawn poses: x, y, z, yaw.
# olive_orchard: headland at the entrance of the path between rows 2 and 3,
# on a flattened pad (values from worlds/olive_orchard.json).
WORLDS = {
    'empty_flat': ('empty_flat.sdf', ('0.0', '0.0', '0.05', '0.0')),
    'olive_orchard': ('olive_orchard.sdf', ('-23.0', '-3.0', '0.533', '0.0')),
}


def _gazebo_and_spawn(context):
    world_arg = LaunchConfiguration('world').perform(context)
    if world_arg in WORLDS:
        world_file = os.path.join(PKG, 'worlds', WORLDS[world_arg][0])
        default_pose = WORLDS[world_arg][1]
    else:
        world_file = world_arg
        default_pose = ('0.0', '0.0', '0.05', '0.0')
    pose = [LaunchConfiguration(k).perform(context) or d
            for k, d in zip(('x', 'y', 'z', 'yaw'), default_pose)]

    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    gz_args = ['-r'] + (['-s'] if headless else []) + [world_file, '--force-version', '8']
    # gz_sim_wrapper.sh stops leftover Gazebo servers from earlier runs and makes
    # sure the server/GUI processes stop together with this launch (see script).
    wrapper = os.path.join(get_package_prefix('agri_ugv_description'),
                           'lib', 'agri_ugv_description', 'gz_sim_wrapper.sh')
    return [
        ExecuteProcess(
            cmd=[wrapper] + gz_args,
            name='gazebo',
            output='screen',
            additional_env={'GZ_SIM_SYSTEM_PLUGIN_PATH': os.pathsep.join(
                [os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', ''),
                 os.environ.get('LD_LIBRARY_PATH', '')])},
            sigterm_timeout='10',
            on_exit=Shutdown(),     # closing the Gazebo window stops everything
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-topic', 'robot_description', '-name', 'agri_ugv',
                       '-x', pose[0], '-y', pose[1], '-z', pose[2], '-Y', pose[3]],
            output='screen',
        ),
    ] + (_global_localization(pose) if world_arg == 'olive_orchard' else []) + (
        _mapping() if world_arg == 'olive_orchard'
        and LaunchConfiguration('slam').perform(context).lower() == 'true' else [])


def _mapping():
    """slam_toolbox on /scan, see config/slam.yaml for the frames."""
    slam_launch = os.path.join(get_package_share_directory('slam_toolbox'),
                               'launch', 'online_async_launch.py')
    return [
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['--frame-id', 'map', '--child-frame-id', 'slam_map'],
            parameters=[{'use_sim_time': True}],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch),
            launch_arguments={'use_sim_time': 'true',
                              'slam_params_file': os.path.join(PKG, 'config', 'slam.yaml')}.items(),
        ),
    ]


def _global_localization(pose):
    """RTK-corrected EKF, map -> odom. It starts from the spawn pose: the map
    frame is the world frame and a single antenna gives no heading at rest."""
    initial = [0.0] * 15
    initial[0], initial[1], initial[5] = float(pose[0]), float(pose[1]), float(pose[3])
    return [
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_global',
            parameters=[EKF_CONFIG, {'use_sim_time': True, 'initial_state': initial}],
            remappings=[('odometry/filtered', '/odometry/global')],
            output='screen',
        ),
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            parameters=[EKF_CONFIG, {'use_sim_time': True}],
            remappings=[('imu', '/imu/data'), ('gps/fix', '/gps/rtk'),
                        ('odometry/filtered', '/odometry/global'),
                        ('odometry/gps', '/odometry/gps')],
            output='screen',
        ),
    ]


def generate_launch_description():
    xacro_file = os.path.join(PKG, 'urdf', 'agri_ugv.urdf.xacro')
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file,
                 ' enable_camera:=', LaunchConfiguration('camera'),
                 ' enable_lidar:=', LaunchConfiguration('lidar')]),
        value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='empty_flat',
                              description='empty_flat, olive_orchard, or a path to an .sdf file'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('camera', default_value='true'),
        DeclareLaunchArgument('lidar', default_value='true'),
        DeclareLaunchArgument('slam', default_value='false'),
        DeclareLaunchArgument('x', default_value=''),
        DeclareLaunchArgument('y', default_value=''),
        DeclareLaunchArgument('z', default_value=''),
        DeclareLaunchArgument('yaw', default_value=''),

        # Lets worlds use model://olive_tree_a, model://orchard_terrain, ...
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(PKG, 'models')),

        OpaqueFunction(function=_gazebo_and_spawn),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
            output='screen',
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{'config_file': os.path.join(PKG, 'config', 'ros_gz_bridge.yaml'),
                         'use_sim_time': True}],
            output='screen',
        ),

        # 2D /scan from the 3D LiDAR cloud (for slam_toolbox / Nav2)
        Node(
            package='agri_ugv_description',
            executable='cloud_to_scan.py',
            parameters=[{'use_sim_time': True,
                         'no_hit_range': ParameterValue(PythonExpression(
                             ['29.0 if "', LaunchConfiguration('slam'), '" == "true" else 0.0']),
                             value_type=float)}],
            condition=IfCondition(LaunchConfiguration('lidar')),
            output='screen',
        ),

        # RTK GNSS: robot GPS corrected by the base station -> /gps/raw, /gps/fix
        Node(
            package='agri_ugv_description',
            executable='rtk_gnss.py',
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),

        # Sensor fusion (config/ekf.yaml): odom -> base_footprint. In olive_orchard
        # _global_localization() adds the RTK-corrected EKF, map -> odom.
        Node(
            package='agri_ugv_description',
            executable='sensor_covariance.py',
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_local',
            parameters=[EKF_CONFIG, {'use_sim_time': True}],
            remappings=[('odometry/filtered', '/odometry/local')],
            output='screen',
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', PythonExpression(
                ['"', os.path.join(PKG, 'config', 'map.rviz'), '" if "',
                 LaunchConfiguration('slam'), '" == "true" else "',
                 os.path.join(PKG, 'config', 'view.rviz'), '"'])],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
