from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    model = LaunchConfiguration('model')
    color_topic = LaunchConfiguration('color_topic')

    declare_model = DeclareLaunchArgument(
        'model',
        default_value='yolov8n_custom.pt',
        description='YOLO model filename stored in the package models folder'
    )

    declare_color_topic = DeclareLaunchArgument(
        'color_topic',
        default_value='/camera/image_raw',
        description='Raw RGB image topic'
    )

    yolo_node = Node(
        package='my_robot_ai_identification',
        executable='yolo_prediction_node_rgb_sw_exec',
        name='yolo_prediction_node',
        output='screen',
        parameters=[{
            'model': model,
            'color_topic': color_topic,
        }]
    )

    return LaunchDescription([
        declare_model,
        declare_color_topic,
        yolo_node
    ])