import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    # ----------------------------
    # Paths
    # ----------------------------
    moveit_pkg = get_package_share_directory('hexapod_moveit_config_new')
    xacro_file = os.path.join(moveit_pkg, 'config', 'hexapod.urdf.xacro')
    srdf_file = os.path.join(moveit_pkg, 'config', 'hexapod.srdf')
    rviz_config = os.path.join(moveit_pkg, 'config', 'moveit.rviz')

    # ----------------------------
    # Launch arguments
    # ----------------------------
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true')

    # ----------------------------
    # Robot Description (URDF)
    # ----------------------------
    robot_description = Command(['xacro ', xacro_file, ' use_gazebo:=true'])
    # Robot Semantic Description (SRDF)
    with open(srdf_file, 'r') as f:
        robot_description_semantic = f.read()

    # ----------------------------
    # Move Group Node
    # ----------------------------
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'robot_description_semantic': robot_description_semantic},
            {'use_sim_time': use_sim_time},
        ]
    )

    # ----------------------------
    # RViz Node
    # ----------------------------
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[
            {'robot_description': robot_description},
            {'robot_description_semantic': robot_description_semantic},
            {'use_sim_time': use_sim_time},
        ]
    )

    # ----------------------------
    # LaunchDescription
    # ----------------------------
    return LaunchDescription([
        move_group_node,
        rviz_node,
    ])
