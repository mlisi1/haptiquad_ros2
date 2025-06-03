import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch import LaunchDescription

def generate_launch_description():


    self_pkg = get_package_share_directory('haptiquad_ros2')

    observer_config = os.path.join(self_pkg, 'config', 'haptiquad_bag.yaml')

    config_file = DeclareLaunchArgument('config_file', default_value=observer_config, description="Observer config file")

    haptiquad = Node(
        package="haptiquad_ros2", executable="haptiquad_bag",
        emulate_tty = True,
        # remappings=[('/robot_description', '/fb/floating_base_description')],
        parameters=[LaunchConfiguration('config_file')]
    )  


    return LaunchDescription(
        [   
            config_file,
            haptiquad
        ]
    )