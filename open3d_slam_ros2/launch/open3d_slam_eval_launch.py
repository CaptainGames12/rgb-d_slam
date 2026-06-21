import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Launch arguments
    seq_file_arg = DeclareLaunchArgument('sequence_file', default_value='', description='Full path to ScanNet sequence file')
    sequences_dir_arg = DeclareLaunchArgument('sequences_dir', default_value='', description='Directory containing multiple sequences')
    sequence_name_arg = DeclareLaunchArgument('sequence_name', default_value='', description='Sequence filename inside sequences_dir')

    default_param = os.path.join(get_package_share_directory('open3d_slam_core'), 'include', 'example_param', 'configuration.lua')
    parameter_file_arg = DeclareLaunchArgument('parameter_file', default_value=default_param, description='Path to Lua parameter file')

    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulated time')
    depth_topic_arg = DeclareLaunchArgument('depth_topic', default_value='/camera/depth/image_raw', description='Depth image topic')
    camera_info_topic_arg = DeclareLaunchArgument('camera_info_topic', default_value='/camera/depth/camera_info', description='CameraInfo topic')

    output_dir_arg = DeclareLaunchArgument('output_dir', default_value='./slam_traj', description='Output directory for trajectories')
    eval_arg = DeclareLaunchArgument('evaluate', default_value='false', description='Run trajectory evaluation after SLAM')

    def _launch(context, *args, **kwargs):
        seq_file = LaunchConfiguration('sequence_file').perform(context)
        sequences_dir = LaunchConfiguration('sequences_dir').perform(context)
        sequence_name = LaunchConfiguration('sequence_name').perform(context)
        if not seq_file and sequence_name:
            seq_file = os.path.join(sequences_dir, sequence_name)

        parameter_file = LaunchConfiguration('parameter_file').perform(context)
        use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
        depth_topic = LaunchConfiguration('depth_topic').perform(context)
        camera_info_topic = LaunchConfiguration('camera_info_topic').perform(context)
        output_dir = LaunchConfiguration('output_dir').perform(context)
        evaluate = LaunchConfiguration('evaluate').perform(context).lower() == 'true'

        # Create output directory
        os.makedirs(output_dir, exist_ok=True)

        # scannet_publisher node
        scannet_params = {}
        if seq_file:
            scannet_params['file'] = seq_file
        scannet_node = Node(
            package='scannet_publisher',
            executable='scannet_publisher',
            name='scannet_publisher',
            output='screen',
            parameters=[scannet_params]
        )

        # open3d_slam node with trajectory output
        slam_params = {
            'parameter_file': parameter_file,
            'depth_topic': depth_topic,
            'camera_info_topic': camera_info_topic,
            'use_sim_time': use_sim_time,
            'output_trajectory_file': os.path.join(output_dir, 'estimated_trajectory.txt')
        }
        slam_node = Node(
            package='open3d_slam_ros2',
            executable='open3d_slam_node',
            name='open3d_slam_node',
            output='screen',
            parameters=[slam_params]
        )

        actions = [scannet_node, slam_node]

        # Optional: trajectory evaluation after SLAM
        if evaluate:
            gt_file = os.path.join(output_dir, 'gt.txt')
            est_file = os.path.join(output_dir, 'estimated_trajectory.txt')
            eval_script = os.path.join(
                get_package_share_directory('open3d_slam_ros2'),
                'evaluate_trajectory.py' # Переконайся, що шлях до скрипта правильний
            )
            eval_process = ExecuteProcess(
                cmd=['python3', eval_script, gt_file, est_file],
                output='screen',
                shell=False
            )
            
            # Запускаємо eval_process ТІЛЬКИ після завершення роботи slam_node
            eval_event = RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=slam_node,
                    on_exit=[eval_process]
                )
            )
            actions.append(eval_event)

        
        return actions

    return LaunchDescription([
        seq_file_arg,
        sequences_dir_arg,
        sequence_name_arg,
        parameter_file_arg,
        use_sim_time_arg,
        depth_topic_arg,
        camera_info_topic_arg,
        output_dir_arg,
        eval_arg,
        OpaqueFunction(function=_launch),
    ])
