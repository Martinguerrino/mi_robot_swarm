# ==============================================================================
# File: view_robot_rviz_.launch.py
# Description: Launch file that publishes the robot_description (URDF) and starts RViz.
#              - Loads the URDF from <pkg_share>/urdf as selected by the 'urdf_name' arg.
#              - Passes the URDF XML content to robot_state_publisher.
#              - Starts rviz2 and loads rviz/mi_robot.rviz if present; otherwise opens RViz empty.
# Usage: ros2 launch mi_robot_description view_robot_rviz_.launch.py [urdf_name:=mi_robot.urdf]
# Notes:  Uses get_package_share_directory when available; falls back to the package source path.
# Author: Martin Guerrino
# Date:   2025-27/11
# License: see package.xml for license information
# ==============================================================================

import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from launch.conditions import IfCondition



def get_package_share_directory_safe(pkg_name: str)->str:
    try:
        return get_package_share_directory(pkg_name)
    except PackageNotFoundError:
        return str(Path(__file__).resolve().parents[1])
        
def generate_launch_description():
    pkg_dir = get_package_share_directory_safe("mi_robot_description")
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        "urdf_name",
        default_value="mi_robot.urdf",
        description="name of the urdf file inside <pkg>/urdf to be loaded"
    ))
    #it creates a robot_state_publisher node to publish the robot description to tf
    #it uses substitutions to get the urdf file path and read its content in launch time
    pkg_dir_s = TextSubstitution(text=pkg_dir)
    urdf_name = LaunchConfiguration("urdf_name")
    urdf_file = PathJoinSubstitution([pkg_dir_s,"urdf",urdf_name])
    urdf_data = Command([FindExecutable(name="cat"), " ",urdf_file])
    ld.add_action(Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": urdf_data}]
    )
    )
    #it creates an rviz2 node to visualize the robot
    rviz_config_file = os.path.join(pkg_dir,"rviz","mi_robot.rviz")
    rviz_args = ["-d",rviz_config_file] if os.path.exists(rviz_config_file) else []
    ld.add_action(Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments = rviz_args
    ))
    ld.add_action(Node(
    package="joint_state_publisher_gui",
    executable="joint_state_publisher_gui",
    name="joint_state_publisher_gui",
    output="screen"
))
    return ld

  


    
