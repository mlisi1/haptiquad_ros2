import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression

from launch import LaunchDescription

def generate_launch_description():


    self_pkg = get_package_share_directory('haptiquad_ros2')

    observer_config = os.path.join(self_pkg, 'config', 'haptiquad_mujoco.yaml')

    config_file = DeclareLaunchArgument('config_file', default_value=observer_config, description="Observer config file")
    mass_scale = DeclareLaunchArgument('mass_scale', default_value='1.0', description="Mass scale factor")
    inertia_scale = DeclareLaunchArgument('inertia_scale', default_value='1.0', description="Inertia scale factor")
    debug = DeclareLaunchArgument('debug', default_value='false', description="Debug mode")


    log_level = PythonExpression([
        " 'DEBUG' if '", LaunchConfiguration('debug'), "' == 'true' else 'INFO' "
    ])

    haptiquad = Node(
        package="haptiquad_ros2", executable="haptiquad_mujoco",
        emulate_tty = True,
        # remappings=[('/robot_description', '/fb/floating_base_description')],
        parameters=[
            LaunchConfiguration('config_file')
        ,   {
                'estimator.mass_scaling': LaunchConfiguration('mass_scale'),
                'estimator.inertia_scaling': LaunchConfiguration('inertia_scale')
            }],
        arguments=[
            '--ros-args',
            '--log-level',
            log_level
        ]
    )  


    return LaunchDescription(
        [   
            config_file,
            mass_scale,
            inertia_scale,
            debug,
            haptiquad
        ]
    )