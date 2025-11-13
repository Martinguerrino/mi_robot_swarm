import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    pkg_path = get_package_share_directory('mi_robot_description')
    xacro_file = os.path.join(pkg_path, 'urdf', 'mi_robot.urdf.xacro')
    robot_description_content = Command(['xacro ', xacro_file])
    rviz_config_file = os.path.join(pkg_path, 'rviz', 'mi_config.rviz')

    # launch-time flags so user can choose to start GUI tools (they may crash
    # on some systems due to library conflicts, so keep them optional)
    use_rviz = LaunchConfiguration('use_rviz')
    use_gui = LaunchConfiguration('use_gui')

    ld = LaunchDescription()

    # declare arguments (default false to avoid launching GUI that may conflict)
    ld.add_action(DeclareLaunchArgument('use_rviz', default_value='false', description='Start RViz2'))
    ld.add_action(DeclareLaunchArgument('use_gui', default_value='false', description='Start joint_state_publisher_gui'))

    # robot_state_publisher - always start
    ld.add_action(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}],
    ))

    # joint_state_publisher_gui - optional (may fail on some systems, run manually in clean shell)
    ld.add_action(Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(use_gui),
    ))

    # rviz2 - optional
    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(use_rviz),
    ))

    return ld