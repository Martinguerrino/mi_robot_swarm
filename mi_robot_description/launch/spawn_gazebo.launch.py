import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch.conditions import IfCondition

def get_package_share_directory_safe(pkg_name: str) -> str:
    try:
        return get_package_share_directory(pkg_name)
    except PackageNotFoundError:
        return str(Path(__file__).resolve().parents[1])

def generate_launch_description():
    pkg_share = get_package_share_directory_safe("mi_robot_description")

    urdf_name_arg = DeclareLaunchArgument(
        "urdf_name",
        default_value="mi_robot.urdf",
        description="URDF file name in package urdf folder"
    )
    robot_name_arg = DeclareLaunchArgument(
        "robot_name",
        default_value="mi_robot",
        description="Name for the spawned entity in Gazebo"
    )
    use_gui_arg = DeclareLaunchArgument(
        "use_gui",
        default_value="true",
        description="Start joint_state_publisher_gui"
    )

    urdf_path = PathJoinSubstitution([pkg_share, "urdf", LaunchConfiguration("urdf_name")])

    # Start Gazebo (classic). Loads factory plugin so spawn_entity works.
    gazebo_cmd = ExecuteProcess(
        cmd=["sudo", "gz", "sim", "--verbose", "-s", "libgazebo_ros_factory.so"],
        output="screen"
    )

    # robot_state_publisher to publish TF from URDF (process xacro at launch time)
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": Command([FindExecutable(name="xacro"), " ", urdf_path])
        }]
    )

    # optional GUI publisher
    jsp_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_gui"))
    )

    # spawn the robot into running Gazebo after a short delay (give Gazebo time to initialize)
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-file", urdf_path, "-entity", LaunchConfiguration("robot_name")],
        output="screen"
    )
    delayed_spawn = TimerAction(period=3.0, actions=[spawn_entity])

    ld = LaunchDescription()
    ld.add_action(urdf_name_arg)
    ld.add_action(robot_name_arg)
    ld.add_action(use_gui_arg)
    ld.add_action(gazebo_cmd)
    ld.add_action(rsp_node)
    ld.add_action(jsp_node)
    ld.add_action(delayed_spawn)

    return ld