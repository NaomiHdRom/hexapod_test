import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():

    pkg_share = os.path.join(os.environ['HOME'], 'ROS2Dev/hexapod_test/src/hexapod_description')

    xacro_file = os.path.join(pkg_share, 'urdf', 'hexapod.xacro')

    # Convierte XACRO a URDF en runtime
    robot_description_config = {'robot_description': os.popen(f"xacro {xacro_file}").read()}

    # Configuración de controladores en línea
    controller_config = {
        'controller_manager': {
            'ros__parameters': {
                'update_rate': 100
            }
        },
        'joint_state_broadcaster': {
            'ros__parameters': {
                'type': 'joint_state_broadcaster/JointStateBroadcaster',
                'publish_rate': 50
            }
        },
        'leg_controller': {
            'ros__parameters': {
                'type': 'joint_trajectory_controller/JointTrajectoryController',
                'joints': [
                    'leg1_jointA', 'leg1_jointB', 'leg1_jointC',
                    'leg2_jointA', 'leg2_jointB', 'leg2_jointC',
                    'leg3_jointA', 'leg3_jointB', 'leg3_jointC',
                    'leg4_jointA', 'leg4_jointB', 'leg4_jointC',
                    'leg5_jointA', 'leg5_jointB', 'leg5_jointC',
                    'leg6_jointA', 'leg6_jointB', 'leg6_jointC'
                ],
                'state_publish_rate': 50,
                'command_interfaces': ['position']
            }
        }
    }

    return LaunchDescription([
        # Lanzar Gazebo vacío con soporte ROS
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # Spawner para el robot
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'hexapod'],
            output='screen'
        ),

        # Nodo ros2_control con controladores
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[robot_description_config, controller_config],
            output='screen'
        ),

        # Lanzar RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'hexapod.rviz')],
            output='screen'
        )
    ])
