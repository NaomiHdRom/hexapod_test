import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
import xacro


def generate_launch_description():

    gazebo = ExecuteProcess(
        cmd=[
            'gazebo', '--verbose',
            '-s', 'libgazebo_ros_factory.so',
            os.path.join(
                get_package_share_directory('hexapod_bringup'),
                'world',
                'hexapod_world.world'
            )
        ],
        output='screen'
    )

    package_path = get_package_share_directory('hexapod_description')
    xacro_file = os.path.join(package_path, 'urdf', 'hexapod_controller.xacro')
    rviz_config_path = os.path.join(package_path, 'rviz', 'urdf.rviz')

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'hexapod'],
        output='screen'
    )

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    hexapod_control = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'hexapod_legs_controller'],
        output='screen'
    )

    # RViz
 

    # 🔥 MoveIt Config NEW
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hexapod_moveit_config_new'),
                'launch',
                'demo.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': 'true'
        }.items()
    )

    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity,

        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_controller]
            )
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[hexapod_control]
            )
        ),

        moveit_launch,
     
    ])
