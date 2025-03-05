import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

from launch import LaunchDescription

def generate_launch_description():


    self_pkg = get_package_share_directory('momobs_ros2')

    observer_config = os.path.join(self_pkg, 'config', 'momobs_mujoco.yaml')

    config_file = DeclareLaunchArgument('config_file', observer_config)

    momobs = Node(
        package="momobs_ros2", executable="momobs_mujoco",
        emulate_tty = True,
        # remappings=[('/robot_description', '/fb/floating_base_description')],
        parameters=[config_file],

    )  


    return LaunchDescription(
        [
            momobs
        ]
    )