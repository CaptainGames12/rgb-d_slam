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
        # 2. Генератор хмари
        Node(
            package='depth_image_proc',
            executable='point_cloud_xyzrgb_node',
            name='PointCloudXyzrgbNode',
            remappings=[
                ('rgb/image_rect_color', '/camera/color/image_raw'),
                ('depth_registered/image_rect', '/camera/depth/image_raw'),
                ('rgb/camera_info', '/camera/color/camera_info')
            ]
        ),

        # 3. GLIM SLAM
        Node(
            package='glim_ros',
            executable='glim_rosnode',
            name='glim_ros',
            remappings=[
                ('/os_cloud_node/points', '/points')
            ]
        )
    ])
