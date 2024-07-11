import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    #orora_launch_file_path = config_param_file_path = os.path.join(
    #    get_package_share_directory('src'),
    #    'nevtech-radar-slam',
    #    'orora',
    #    'launch',
    #    'run_orora.launch.py'
    #    )
    #'/home/lukas/ros2_slam_ws/src/navtech-radar-slam/orora/launch/run_orora.launch.py'
    #sc_pgo_launch_file_path = config_param_file_path = os.path.join(
    #    get_package_share_directory('src'),
    #    'nevtech-radar-slam',
    #    'pgo',
    #    'SC-A-LOAM',
    #    'launch',
    #    'sc_pgo.launch.py'
    #    )
    #'/home/lukas/ros2_slam_ws/src/navtech-radar-slam/pgo/SC-A-LOAM/launch/sc_pgo.launch.py'

    #orora_launch_file_path = get_package_share_directory('nevtech-radar-slam') + '/orora/launch/run_orora.launch.py'
    #sc_pgo_launch_file_path = get_package_share_directory('nevtech-radar-slam') + '/pgo/SC-A-LOAM/launch/sc_pgo.launch.py'

    orora_launch_file_path = '/home/lukas/ros2_slam_ws/src/navtech-radar-slam/orora/launch/run_orora.launch.py'
    sc_pgo_launch_file_path = '/home/lukas/ros2_slam_ws/src/navtech-radar-slam/pgo/SC-A-LOAM/launch/sc_pgo.launch.py'

    orora_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(orora_launch_file_path)
    )
    
    pgo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sc_pgo_launch_file_path)
    )

    return LaunchDescription([
        orora_launch,
        pgo_launch
    ])

        