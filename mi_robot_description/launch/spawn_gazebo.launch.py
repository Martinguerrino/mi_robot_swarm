import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from launch.conditions import IfCondition

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

    # --- Rutas a los archivos ---
    # Ruta al archivo XACRO
    xacro_file_path = PathJoinSubstitution([pkg_share, "xacro", LaunchConfiguration("urdf_name")])
    
    # Ruta al archivo de parámetros del bridge
    bridge_params_path = PathJoinSubstitution([pkg_share, "parameters", "bridge_parameters.yaml"])

    # --- Comandos y Nodos ---
    # Comando para procesar el XACRO y obtener la descripción del robot
    robot_description_content = Command([
        FindExecutable(name="xacro"), " ", xacro_file_path
    ])

    # 1. Iniciar Gazebo (gz sim)
    gz_sim_cmd = ExecuteProcess(
        cmd=[FindExecutable(name="gz"), "sim", "-r", "-v4"],
        output="screen"
    )

    # 2. Publicador del estado del robot (robot_state_publisher)
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description_content}]
    )

    # 3. Publicador de estado de las articulaciones (opcional)
    jsp_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_gui"))
    )

    # 4. Nodo para "spawnear" (crear) el robot en Gazebo
    spawn_entity_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "robot_description",
            "-name", LaunchConfiguration("robot_name"),
            "-allow_renaming", "true"
        ],
        output="screen"
    )
    
    # 5. Puente para comunicar tópicos entre Gazebo y ROS 2
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        parameters=[{'config_file': bridge_params_path}],
        output='screen'
    )

    # --- Ensamblaje del LaunchDescription ---
    ld = LaunchDescription()
    ld.add_action(urdf_name_arg)
    ld.add_action(robot_name_arg)
    ld.add_action(use_gui_arg)
    ld.add_action(gz_sim_cmd)
    ld.add_action(rsp_node)
    ld.add_action(jsp_node)
    ld.add_action(bridge_node)
    
    # Se usa un manejador de eventos para asegurar que el robot se "spawnea"
    # solo después de que Gazebo se haya iniciado.
    ld.add_action(RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gz_sim_cmd,
            on_exit=[spawn_entity_node],
        )
    ))

    return ld