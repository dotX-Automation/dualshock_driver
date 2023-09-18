"""
Joystick app launch file.

Lorenzo Bianchi <lnz.bnc@gmail.com>
Roberto Masocco <robmasocco@gmail.com>
Intelligent Systems Lab <isl.torvergata@gmail.com>

January 11, 2022
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """Builds a LaunchDescription for the Joy2cmdvel app"""
    ld = LaunchDescription()

    # Build config file path
    config = os.path.join(get_package_share_directory('joystick'), 'config/joystick.yaml')

    # Declare launch arguments
    ns = LaunchConfiguration('namespace')
    cf = LaunchConfiguration('cf')
    ns_launch_arg = DeclareLaunchArgument(
        'namespace',
        default_value='joystick')
    cf_launch_arg = DeclareLaunchArgument(
        'cf',
        default_value=config)
    ld.add_action(ns_launch_arg)
    ld.add_action(cf_launch_arg)

    # Create node launch description
    node = Node(
        package='joystick',
        executable='joystick_app',
        shell=False,
        emulate_tty=True,
        output='both',
        log_cmd=True,
        namespace=ns,
        parameters=[cf]
    )

    ld.add_action(node)

    return ld
