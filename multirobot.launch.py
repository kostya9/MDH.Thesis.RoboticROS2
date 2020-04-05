import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    gzros_dir = get_package_share_directory('gazebo_ros')
    return launch.LaunchDescription([
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
            gzros_dir + '/launch/gazebo.launch.py'), launch_arguments=[
                ("world", "./world.world"),
                ("verbose", "true")
            ]),
        launch.actions.GroupAction([
            launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource('./onerobot.launch.py'), launch_arguments=[
                ("robot_name", "robot_0"),
                ("init_pose_x", "-12.0"),
                ("init_pose_y", "-12.0"),
                ("init_pose_z", "0")
            ]),
        ]),
    ])