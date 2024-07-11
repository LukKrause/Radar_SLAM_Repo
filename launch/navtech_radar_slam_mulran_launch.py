from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    orora_launch_file = '/home/lukas/ros2_slam_ws/src/navtech-radar-slam/outlier-robust-radar-odometry/launch/run_orora_launch.py'
    sc_pgo_launch_file = '/home/lukas/ros2_slam_ws/src/navtech-radar-slam/pgo/SC-A-LOAM/launch/sc_pgo_launch.py'

    return LaunchDescription([
        DeclareLaunchArgument('seq_dir', default_value='/home/lukas/Downloads/KAIST_Dataset/KAIST03'),
        DeclareLaunchArgument('do_slam', default_value='true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(orora_launch_file),
            launch_arguments={
                'seq_dir': LaunchConfiguration('seq_dir'),
                'do_slam': LaunchConfiguration('do_slam')
            }.items()
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sc_pgo_launch_file)
        )
    ])
