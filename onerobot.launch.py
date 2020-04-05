import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from launch.substitutions import LaunchConfiguration 
import json

def generate_launch_description():
    entity_xml = open('./models/my_robot/model.sdf', 'r').read()

    list_subs = [
        "{'name':",
        LaunchConfiguration("robot_name"),
        ", 'xml': " + json.encoder.py_encode_basestring(entity_xml),
        ", 'initial_pose': {'position': { 'x':",
        LaunchConfiguration('init_pose_x'),
        ",'y':",
        LaunchConfiguration('init_pose_y'),
        ",'z':",
        LaunchConfiguration('init_pose_z'),
        "} } }"
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
        launch.actions.ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/spawn_entity', 'gazebo_msgs/SpawnEntity', list_subs],
            output='screen')
    ])