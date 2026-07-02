"""Launch the UM7 driver with YAML params and an optional ``port`` override."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    """Build the node action, letting ``port:=`` win over the YAML file."""
    params_file = LaunchConfiguration('params_file').perform(context)
    port = LaunchConfiguration('port').perform(context)

    # Later entries override earlier ones, so the CLI override wins.
    parameters = [params_file]
    if port:
        parameters.append({'port': port})

    return [Node(
        package='um7_driver',
        executable='um7_node',
        name='um7_node',
        output='screen',
        parameters=parameters,
    )]


def generate_launch_description():
    """Declare launch arguments and defer node construction to _launch_setup."""
    default_params = os.path.join(
        get_package_share_directory('um7_driver'), 'config', 'um7.yaml')
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='Path to the parameters YAML file.'),
        DeclareLaunchArgument(
            'port', default_value='',
            description='Serial port override; wins over the YAML file when set.'),
        OpaqueFunction(function=_launch_setup),
    ])
