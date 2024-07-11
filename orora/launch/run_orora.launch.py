import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    #config_param_file_path = os.path.join(
    #    get_package_share_directory('src'),
    #    'nevtech-radar-slam',
    #    'orora',
    #    'config',
    #    'orora_params.yaml'
    #    )
    #'home/lukas/ros2_slam_ws/src/navtech-radar-slam/orora/config/orora_params.yaml'
    #rviz_file_path = os.path.join(
    #    get_package_share_directory('src'),
    #    'nevtech-radar-slam',
    #    'orora',
    #    'rviz',
    #    'orora_viz.rviz'
    #    )

    #config_param_file_path = get_package_share_directory('nevtech-radar-slam') + '/orora/config/orora_params.yaml'
    #rviz_file_path = get_package_share_directory('nevtech-radar-slam') + '/orora/rviz/orora_viz.rviz'

    config_param_file_path = '/home/lukas/ros2_slam_ws/src/navtech-radar-slam/orora/config/orora_params.yaml'
    rviz_file_path = 'home/lukas/ros2_slam_ws/src/navtech-radar-slam/orora/rviz/orora_viz.rviz'
    # Declare arguments
    seq_dir_arg = DeclareLaunchArgument('seq_dir', default_value='/home/lukas/Downloads/KAIST_Dataset/KAIST03')
    do_slam_arg = DeclareLaunchArgument('do_slam', default_value='true')
    rviz_arg = DeclareLaunchArgument('rviz', default_value='true')

    return LaunchDescription([
            seq_dir_arg,
            do_slam_arg,
            rviz_arg,

            Node(
                package='orora',
                executable='orora_odom',
                name='orora_node',
                output='screen',
                parameters=[
                    config_param_file_path,
                    {'seq_dir': LaunchConfiguration('seq_dir')},
                    {'do_slam': LaunchConfiguration('do_slam')},
                    {'keypoint_extraction': 'cen2018'},
                    {'algorithm': 'ORORA'},
                    {'dataset': 'mulran'},
                    {'viz_extraction': False},
                    {'viz_matching': False},
                    {'frame_rate': 4},
                    {'stop_each_frame': False}
                ]
            ),

            # Define RViz node
            Node(
                package='rviz2',
                executable='rviz2',
                name='orora_rviz_node',
                output='screen',
                condition=IfCondition(LaunchConfiguration('rviz')),
                arguments=['-d', rviz_file_path]
            )
        ])
    