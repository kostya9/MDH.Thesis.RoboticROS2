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
    entity_xml = open('./models/my_robot/model.sdf', 'r').read()



    list_subs = [
        "{",
            "'name':", LaunchConfiguration("robot_name"), ",",
            "'xml':" + json.encoder.py_encode_basestring(entity_xml), ",",
            "'robot_namespace':", LaunchConfiguration('robot_name'), ",",
            "'initial_pose':",
            "{",
                "'position':", 
                "{",
                    "'x':", LaunchConfiguration('init_pose_x'), ",",
                    "'y':", LaunchConfiguration('init_pose_y'), ",",
                    "'z':", LaunchConfiguration('init_pose_z'), ",",
                "},",
                "'orientation':",
                "{",
                    "'x':", LaunchConfiguration("x_quaternion"), ",",
                    "'y':", LaunchConfiguration("y_quaternion"), ",",
                    "'z':", LaunchConfiguration("z_quaternion"), ",",
                    "'w':", LaunchConfiguration("w_quaternion"), ",",
                "}",
            "}",
        "}"
    ]

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'robot_name'),
        launch.actions.DeclareLaunchArgument(
            'init_pose_x'),
            launch.actions.DeclareLaunchArgument(
            'init_pose_y'),
            launch.actions.DeclareLaunchArgument(
            'init_pose_z'),
            launch.actions.DeclareLaunchArgument(
            'x_quaternion'),
            launch.actions.DeclareLaunchArgument(
            'y_quaternion'),
            launch.actions.DeclareLaunchArgument(
            'z_quaternion'),
            launch.actions.DeclareLaunchArgument(
            'w_quaternion'),
        launch.actions.ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/spawn_entity', 'gazebo_msgs/SpawnEntity', list_subs],
            output='screen')
    ])