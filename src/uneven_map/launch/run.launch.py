#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_share = get_package_share_directory('uneven_map')
    
    default_params_file = os.path.join(pkg_share, 'config', 'param.yaml')
    
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Path to the YAML parameter file'
    )


    uneven_map_node = Node(
        package='uneven_map',
        executable='uneven_map_node',
        name='uneven_map_node',
        output='screen',
        emulate_tty=True,
        parameters=[LaunchConfiguration('params_file')],
    )
    
    return LaunchDescription([
        params_file_arg,
        uneven_map_node,
    ])