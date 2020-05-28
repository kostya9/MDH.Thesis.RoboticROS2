import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from launch.substitutions import LaunchConfiguration 
from launch.frontend.parse_substitution import parse_substitution
import json
import math
import sys

def generate_launch_description():
    x = LaunchConfiguration('init_pose_x')
    y = LaunchConfiguration('init_pose_y')
    z = LaunchConfiguration('init_pose_z')
    yaw = LaunchConfiguration('init_pose_yaw')
    entity_name = LaunchConfiguration('entity_name')
    path = './models/brick_box_3x1x3/model.sdf'

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'init_pose_x'),
            launch.actions.DeclareLaunchArgument(
            'init_pose_y'),
            launch.actions.DeclareLaunchArgument(
            'init_pose_z'),
            launch.actions.DeclareLaunchArgument(
            'init_pose_yaw'),
        launch.actions.ExecuteProcess(
            cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py', '-entity', entity_name, '-x', x, '-y', y, '-z', z, '-Y', yaw, '-file', path],
            output='screen')
    ])