import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression

from launch import LaunchDescription

def generate_launch_description():


    self_pkg = get_package_share_directory('haptiquad_ros2')

    observer_config = os.path.join(self_pkg, 'config', 'haptiquad_mujoco.yaml')
    default_burst_config = os.path.join(self_pkg, 'config', 'null_burst_config.yaml')

    config_file = DeclareLaunchArgument('config_file', default_value=observer_config, description="Observer config file")
    burst_config = DeclareLaunchArgument('burst_config', default_value=default_burst_config, description="Burst config file")
    mass_scale = DeclareLaunchArgument('mass_scale', default_value='1.0', description="Mass scale factor")
    inertia_scale = DeclareLaunchArgument('inertia_scale', default_value='1.0', description="Inertia scale factor")
    debug = DeclareLaunchArgument('debug', default_value='false', description="Debug mode")
    drop_prob = DeclareLaunchArgument('drop_prob', default_value='0.0', description="Message drop probability")


    log_level = PythonExpression([
        " 'DEBUG' if '", LaunchConfiguration('debug'), "' == 'true' else 'INFO' "
    ])

    haptiquad = Node(
        package="haptiquad_ros2", executable="haptiquad_mujoco",
        emulate_tty = True,
        # remappings=[('/robot_description', '/fb/floating_base_description')],
        parameters=[
            LaunchConfiguration('config_file'),
            LaunchConfiguration('burst_config')
        ,   {
                'evalutation.mass_scaling': LaunchConfiguration('mass_scale'),
                'evalutation.inertia_scaling': LaunchConfiguration('inertia_scale'),
                'evalutation.drop_prob': LaunchConfiguration('drop_prob')
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
            drop_prob,
            burst_config,
            debug,
            haptiquad
        ]
    )