import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    # 1. Оголошення аргументів
    seq_file_arg = DeclareLaunchArgument('sequence_file', default_value='', description='Full path to ScanNet sequence file')
    sequences_dir_arg = DeclareLaunchArgument('sequences_dir', default_value='', description='Directory containing multiple sequences')
    sequence_name_arg = DeclareLaunchArgument('sequence_name', default_value='', description='Sequence filename inside sequences_dir')

    default_param = os.path.join(get_package_share_directory('open3d_slam_core'), 'example_param', 'configuration.lua')
    parameter_file_arg = DeclareLaunchArgument('parameter_file', default_value=default_param, description='Path to Lua parameter file')

    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulated time')
    depth_topic_arg = DeclareLaunchArgument('depth_topic', default_value='/camera/depth/image_raw', description='Depth image topic')
    camera_info_topic_arg = DeclareLaunchArgument('camera_info_topic', default_value='/camera/depth/camera_info', description='CameraInfo topic')

    # НОВІ ПАРАМЕТРИ ДЛЯ ФАЙЛІВ ТРАЄКТОРІЙ
    output_gt_file_arg = DeclareLaunchArgument(
        'output_gt_file',
        default_value='gt.txt',
        description='Full path for Ground Truth file'
    )
    output_trajectory_file_arg = DeclareLaunchArgument(
        'output_trajectory_file',
        default_value='estimated_trajectory.txt',
        description='Full path for Estimated Trajectory file'
    )

    evaluate_arg = DeclareLaunchArgument('evaluate', default_value='false', description='Run evaluation script after SLAM')

    # 2. Формування словників з параметрами для нод
    scannet_params = {
        'file': LaunchConfiguration('sequence_file'),
        'depth_topic': LaunchConfiguration('depth_topic'),
        'camera_info_topic': LaunchConfiguration('camera_info_topic'),
        'output_gt_file': LaunchConfiguration('output_gt_file') # Передаємо шлях у Python-паблішер
    }

    slam_params = {
        'parameter_file': LaunchConfiguration('parameter_file'),
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'output_trajectory_file': LaunchConfiguration('output_trajectory_file') # Передаємо шлях у C++ ядро
    }

    # 3. Ініціалізація нод
    scannet_node = Node(
        package='scannet_publisher', # Переконайся, що назва пакета збігається з твоєю
        executable='scannet_publisher',
        name='scannet_publisher',
        output='screen',
        parameters=[scannet_params]
    )

    slam_node = Node(
        package='open3d_slam_ros2',
        executable='open3d_slam_node',
        name='open3d_slam_node',
        output='screen',
        parameters=[slam_params]
    )

    actions = [scannet_node, slam_node]

    # 4. Скрипт евалюації (Запускається лише після завершення slam_node)
    eval_script = os.path.join(
        get_package_share_directory('open3d_slam_ros2'),
        'evaluate_trajectory.py'
    )

    eval_process = ExecuteProcess(
        cmd=['python3', eval_script, LaunchConfiguration('output_gt_file'), LaunchConfiguration('output_trajectory_file')],
        output='screen',
        shell=False,
        condition=IfCondition(LaunchConfiguration('evaluate'))
    )

    eval_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=slam_node,
            on_exit=[eval_process]
        )
    )
    actions.append(eval_event)

    return LaunchDescription([
        seq_file_arg,
        sequences_dir_arg,
        sequence_name_arg,
        parameter_file_arg,
        use_sim_time_arg,
        depth_topic_arg,
        camera_info_topic_arg,
        output_gt_file_arg,          # Додано
        output_trajectory_file_arg,  # Додано
        evaluate_arg,
        *actions
    ])
