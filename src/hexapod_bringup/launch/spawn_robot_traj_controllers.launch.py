from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, TimerAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.events import TimerEvent
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    ld= LaunchDescription()
    robot_description_file = os.path.join(
        get_package_share_directory('hexapod_description'),'urdf','hexapod.urdf.xacro'
    )
    joint_controllers_file = os.path.join(
        get_package_share_directory('hexapod_moveit_config_new'),'config', 'ros2_controlleers.yaml'
    )
    gazebo_launch_file = os.path.join(
        get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'
    )