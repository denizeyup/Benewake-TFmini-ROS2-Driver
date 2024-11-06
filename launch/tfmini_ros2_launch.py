from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tfmini_ros2',
            executable='tfmini_ros_node',
            name='tfmini_ros_node',
            output='screen',
            parameters=[{
                'serial_port': '/your/port',
                'baud_rate': 115200
            }]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='broadcasterTFmini',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'TFmini']
        )
    ])
