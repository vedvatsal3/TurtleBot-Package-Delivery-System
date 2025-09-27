from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='secoro2_delivery_system', executable='delivery_bt_server', output='screen'),
    ])
