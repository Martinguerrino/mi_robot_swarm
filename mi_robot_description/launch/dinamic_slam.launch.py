from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    ekf_params_path = os.path.join(
        get_package_share_directory('mi_robot_description'),
        'parameters',
        'ekf_params.yaml'
    )
# 2 - SLAM Toolbox
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('slam_toolbox'),
            '/launch/online_async_launch.py'
        ])
    )

    # 3 - Nav2
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('nav2_bringup'),
            '/launch/navigation_launch.py'
        ]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # 4 - Explore Lite
    explore = Node(
        package='explore_lite',
        executable='explore',
        name='explore',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    # 5 - EKF Localization
    ekf = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_params_path, {'use_sim_time': True}]
        )
    ld = LaunchDescription()
    ld.add_action(slam)
    ld.add_action(nav2)
    ld.add_action(explore)
    ld.add_action(ekf)
    return ld
