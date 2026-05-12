#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    model_arg = DeclareLaunchArgument(
        'modelYolo',
        default_value='yolov8n_custom.pt',
        description='YOLO model filename inside models/'
    )

    topic_arg = DeclareLaunchArgument(
        'topic',
        default_value='/image_raw',
        description='Image topic'
    )

    front_distance_arg = DeclareLaunchArgument(
        'front_distance',
        default_value='0.5',
        description='Maximum valid distance to sign in meters'
    )

    signs_file_arg = DeclareLaunchArgument(
        'signs_file',
        default_value='sign_positions_real.yaml',
        description='Signs YAML filename inside config/'
    )

    signs_file = PathJoinSubstitution([
        FindPackageShare('my_robot_ai_identification'),
        'config',
        LaunchConfiguration('signs_file')
    ])

    node = Node(
        package='my_robot_ai_identification',
        executable='rubot_identification_yolo_cls_exec',
        name='object_detection',
        output='screen',
        parameters=[{
            'modelYolo': LaunchConfiguration('modelYolo'),
            'topic': LaunchConfiguration('topic'),
            'front_distance': LaunchConfiguration('front_distance'),
            'signs_file': signs_file,
        }]
    )

    return LaunchDescription([
        model_arg,
        topic_arg,
        front_distance_arg,
        signs_file_arg,
        node
    ])