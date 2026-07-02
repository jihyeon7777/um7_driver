"""Launch the UM7 driver (with TF) and RViz preconfigured for orientation."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    """Start the node with publish_tf on plus RViz using the shipped config."""
    pkg = get_package_share_directory('um7_driver')
    port = LaunchConfiguration('port').perform(context)

    parameters = [os.path.join(pkg, 'config', 'um7.yaml'), {'publish_tf': True}]
    if port:
        parameters.append({'port': port})

    return [
        Node(package='um7_driver', executable='um7_node', name='um7_node',
             output='screen', parameters=parameters),
        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
             arguments=['-d', os.path.join(pkg, 'rviz', 'um7.rviz')]),
    ]


def generate_launch_description():
    """Declare the port override and defer to _launch_setup."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'port', default_value='',
            description='Serial port override; wins over the YAML file when set.'),
        OpaqueFunction(function=_launch_setup),
    ])
