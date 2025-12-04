import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.actions import ExecuteProcess

# ==============================================================================
#<xacro:property name="pi" value="3.141592653589793"/>
# ==============================================================================
def get_package_share_directory_safe(pkg_name: str) -> str:
    try:
        return get_package_share_directory(pkg_name)
    except PackageNotFoundError:
        # Si no se encuentra el paquete (porque no se ha instalado con `colcon build`),
        # se asume que el launch se está ejecutando desde el directorio `src`.
        return str(Path(__file__).resolve().parents[1])

def generate_launch_description():
    # Obtenemos la ruta al paquete de la manera segura
    pkg_share = get_package_share_directory_safe("mi_robot_description")
    
    # --- Argumentos de Lanzamiento ---
    # Argumento para especificar el archivo XACRO a usar
    urdf_name_arg = DeclareLaunchArgument(
        "urdf_name",
        default_value="mi_robot.urdf.xacro",
        description="Nombre del archivo XACRO dentro de la carpeta 'xacro'"
    )
    # Argumento para el nombre del robot en Gazebo
    robot_name_arg = DeclareLaunchArgument(
        "robot_name",
        default_value="mi_robot",
        description="Nombre para la entidad spawneada en Gazebo"
    )
    # Argumento para usar o no el GUI de los joints
    use_gui_arg = DeclareLaunchArgument(
        "use_gui",
        default_value="true",
        description="Iniciar joint_state_publisher_gui"
    )
    start_gz_arg = DeclareLaunchArgument(
        "start_gz",
        default_value="true",
        description="Arrancar gz sim con mi_mundo.sdf"
    )

    # --- Rutas a los archivos ---
    # Ruta al archivo XACRO
    xacro_file_path = PathJoinSubstitution([pkg_share, "xacro", LaunchConfiguration("urdf_name")])
    
    # Ruta al archivo de parámetros del bridge
    bridge_params_path = PathJoinSubstitution([pkg_share, "parameters", "bridge_parameters.yaml"])
    world_path = PathJoinSubstitution([pkg_share, "gazebo", "mi_mundo.sdf"])

    # --- Comandos y Nodos ---
    # Comando para procesar el XACRO y obtener la descripción del robot
    robot_description_content = Command([
        FindExecutable(name="xacro"), " ", xacro_file_path
    ])

    # 1. Publicador del estado del robot (robot_state_publisher)
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description_content}]
    )

    # 2. Publicador de estado de las articulaciones (opcional)
    jsp_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_gui"))
    )

    # 3. Nodo para "spawnear" (crear) el robot en Gazebo
    spawn_entity_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "robot_description",
            "-name", LaunchConfiguration("robot_name"),
            "-allow_renaming", "false"
        ],
        output="screen",
        respawn=False,
        respawn_delay=3.0,
    )
    
    # 4. Puente para comunicar tópicos entre Gazebo y ROS 2
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        parameters=[{'config_file': bridge_params_path}],
        output='screen'
    )

    # 5. Nodo para arrancar Gazebo
    gz_node = ExecuteProcess(
        cmd=[FindExecutable(name="gz"), "sim", "-r", "-v4", world_path],
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_gz"))
    )
    static_lidar = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    arguments=[
        "0", "0", "0", "0", "0", "0",
        "lidar_link",                      # parent (frame del URDF)
        "mi_robot/robot_root/lidar_sensor" # child (el frame del scan)
    ],
    output="screen"
)

    # --- Ensamblaje del LaunchDescription ---
    ld = LaunchDescription()
    ld.add_action(urdf_name_arg)
    ld.add_action(robot_name_arg)
    ld.add_action(use_gui_arg)
    ld.add_action(start_gz_arg)
    ld.add_action(gz_node)
    ld.add_action(rsp_node)
    ld.add_action(jsp_node)
    ld.add_action(static_lidar)
    ld.add_action(bridge_node)
    ld.add_action(TimerAction(period=3.0, actions=[spawn_entity_node]))
    
    return ld