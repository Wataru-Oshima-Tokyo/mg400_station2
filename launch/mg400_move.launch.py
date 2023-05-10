from pathlib import Path
from typing import Dict
import os
import subprocess
import time

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.substitutions import FindExecutable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node



def _load_robot_description(filepath: Path) -> Dict:
    """Load robot description."""
    if 'xacro' in filepath.suffix:
        robot_description_content = Command(
            [
                'xacro',
                ' ',
                str(filepath)
            ],
        )
    else:
        try:
            with open(str(filepath), 'r') as file:
                robot_description_content = file
        except EnvironmentError:
            exit(1)
    return {'robot_description': robot_description_content}

def generate_launch_description():

    this_package_path = get_package_share_path('mg400_bringup')

    ns_arg = DeclareLaunchArgument(
        'namespace', default_value='mg400')
    ns = LaunchConfiguration('namespace')

    joy_arg = DeclareLaunchArgument(
        'joy',
        default_value='false',
        description='Determines if joy.launch is called.')
    joy = LaunchConfiguration('joy')

    ip_address_arg = DeclareLaunchArgument(
        'ip_address', default_value='192.168.1.6')
    
    ip_address = LaunchConfiguration('ip_address')

    mg400_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(this_package_path
                / 'launch'
                / 'mg400.launch.py')),
        launch_arguments=[
            ('namespace', ns),
            ('ip_address', ip_address),
        ])

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=ns,
        output='log',
        parameters=[
            _load_robot_description(
                get_package_share_path('mg400_description') /
                'urdf' / 'mg400.urdf.xacro')])

    joy_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(this_package_path
                / 'launch'
                / 'joy.launch.py')),
        condition=IfCondition(joy),
        launch_arguments=[
            ('namespace', ns)
        ])

    move_command = Node(
        package='mg400_station2',
        executable='mg400_station',
        namespace=ns,
        output='log')

    ld = LaunchDescription()
    ld.add_action(ns_arg)
    ld.add_action(joy_arg)
    ld.add_action(ip_address_arg)
    ld.add_action(move_command)
    ld.add_action(mg400_node)
    ld.add_action(rsp_node)
    ld.add_action(joy_node)
    return ld

