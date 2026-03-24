#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    wp_name_arg = DeclareLaunchArgument(
        'wp_name',
        default_value='waypoints_sw.yaml',
        description='Waypoint YAML filename inside package config folder'
    )

    wp_file = PathJoinSubstitution([
        FindPackageShare('my_robot_nav_control'),
        'config',
        LaunchConfiguration('wp_name')
    ])

    nav_waypoints_node = Node(
        package='my_robot_nav_control',
        executable='nav_waypoints_exec',
        name='nav_waypoints_node',
        output='screen',
        parameters=[
            {'wp_file': wp_file}
        ]
    )

    return LaunchDescription([
        wp_name_arg,
        nav_waypoints_node
    ])