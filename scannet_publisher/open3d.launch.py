from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Ваша камера
        Node(
            package='scannet_publisher',
            executable='scannet_publisher',
            name='scannet_publisher',
            parameters=[{
                'file': "/home/captaingames/Документи/scannet/scans/scene0010_01/scene0010_01.sens"
            }]
        ),

        Node(
            package='rgb_d_slam',
            executable='open_3d',
            name='open_3d',

        )
    ])
