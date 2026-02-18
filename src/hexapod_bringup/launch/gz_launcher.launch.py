import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    # ----------------------------
    # Paths
    # ----------------------------
    bringup_pkg = get_package_share_directory('hexapod_bringup')
    moveit_pkg = get_package_share_directory('hexapod_moveit_config_new')  # solo para URDF

    xacro_file = os.path.join(moveit_pkg, 'config', 'hexapod.urdf.xacro')
    world_file = os.path.join(bringup_pkg, 'world', 'hexapod_world.world')

    # ----------------------------
    # Procesar XACRO
    # ----------------------------
    robot_description_xml = xacro.process_file(
        xacro_file,
        mappings={'use_gazebo': 'true'}
    ).toxml()

    # ----------------------------
    # Robot State Publisher
    # ----------------------------
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description_xml},
            {'use_sim_time': True}
        ]
    )

    # ----------------------------
    # Gazebo
    # ----------------------------
    gazebo = ExecuteProcess(
        cmd=[
            'gazebo',
            '--verbose',
            '-s', 'libgazebo_ros_factory.so',
            world_file
        ],
        output='screen'
    )

    # ----------------------------
    # Spawn robot
    # ----------------------------
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'hexapod'
        ],
        output='screen'
    )

    spawn_entity_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=gazebo,
            on_start=[spawn_entity],
        )
    )

    # ----------------------------
    # Controllers
    # ----------------------------
    controller_spawners = [
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                f'leg{i}_controller',
                '--controller-manager',
                '/controller_manager'
            ],
            output='screen'
        )
        for i in range(1, 7)
    ]

    # ----------------------------
    # Launch final
    # ----------------------------
    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity_handler,
        *controller_spawners,
    ])
