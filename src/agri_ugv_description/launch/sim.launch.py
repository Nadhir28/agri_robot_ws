"""Launch Gazebo Sim with the agri_ugv robot, the ROS<->Gazebo bridge and
robot_state_publisher.

Arguments:
  world:=empty_flat|olive_orchard|<path.sdf>   Default: empty_flat
  headless:=true|false   run the Gazebo server only (no GUI). Default false.
  rviz:=true|false       also start RViz. Default false.
  camera:=true|false     simulate the front camera (rendering, ~RAM). Default true.
  lidar:=true|false      simulate the 3D LiDAR (+ derived /scan). Default true.
  x, y, z, yaw           spawn pose. Default: the selected world's safe spawn
                         pose (see WORLDS below; 0 0 0.05 0 for a custom path).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PKG = get_package_share_directory('agri_ugv_description')

# Named worlds and their default (collision-free) spawn poses: x, y, z, yaw.
# olive_orchard: headland at the entrance of the path between rows 2 and 3,
# on a flattened pad (values from worlds/olive_orchard.json).
WORLDS = {
    'empty_flat': ('empty_flat.sdf', ('0.0', '0.0', '0.05', '0.0')),
    'olive_orchard': ('olive_orchard.sdf', ('-23.0', '-3.0', '0.155', '0.0')),
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
    gz_args = f'-r {"-s " if headless else ""}{world_file}'
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items(),
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-topic', 'robot_description', '-name', 'agri_ugv',
                       '-x', pose[0], '-y', pose[1], '-z', pose[2], '-Y', pose[3]],
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
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('lidar')),
            output='screen',
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(PKG, 'config', 'view.rviz')],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
