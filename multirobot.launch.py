import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
import math


robotNum = 0

def euler_to_quaternion(roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)

        return [qx, qy, qz, qw]

def launch_robot(x, y, rot = 0.0):
    global robotNum
    namespace = "robot_" + str(robotNum)

    q = euler_to_quaternion(0, 0, rot)

    cfg = launch.actions.GroupAction([
            launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource('./onerobot.launch.py'), launch_arguments=[
                ("robot_name", namespace),
                ("init_pose_x", str(x)),
                ("init_pose_y", str(y)),
                ("init_pose_z", "0"),
                ('rot', str(rot)),
                ("x_quaternion", str(q[0])),
                ("y_quaternion", str(q[1])),
                ("z_quaternion", str(q[2])),
                ("w_quaternion", str(q[3]))
            ]),
            launch_ros.actions.Node(node_executable="main", package="pathfinder", node_namespace=namespace)
        ])
    robotNum += 1
    return cfg

def generate_launch_description():
    gzros_dir = get_package_share_directory('gazebo_ros')
    return launch.LaunchDescription([
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
            gzros_dir + '/launch/gazebo.launch.py'), launch_arguments=[
                ("verbose", "true")
            ]),
        launch_robot(3, 5),
        launch_robot(6, 5, rot=3.14),
        launch_ros.actions.Node(node_executable="main", package="pathfinder_gui")
    ])