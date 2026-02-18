import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    # ----------------------------
    # Paths
    # ----------------------------
    bringup_pkg = get_package_share_directory('hexapod_bringup')
    moveit_pkg = get_package_share_directory('hexapod_moveit_config_new')

    rviz_config = os.path.join(moveit_pkg, 'config', 'moveit.rviz')
    xacro_file = os.path.join(moveit_pkg, 'config', 'hexapod.urdf.xacro')
    world_file = os.path.join(bringup_pkg, 'world', 'hexapod_world.world')

    # ----------------------------
    # Procesar XACRO (FORMA CORRECTA)
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
    # MoveIt
    # ----------------------------
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_pkg, 'launch', 'demo.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # ----------------------------
    # TF world -> base_footprint
    # ----------------------------
    world_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_footprint'],
        output='screen'
    )

    # ----------------------------
    # RViz (CORRECTO)
    # ----------------------------
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
    )

    # ----------------------------
    # Launch final
    # ----------------------------
    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity_handler,
        *controller_spawners,
        moveit_launch,
        world_tf,
        rviz_node,
    ])
