from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sc_pgo',
            executable='alaserPGO',
            name='pgo_node',
            output='screen',
            parameters=[
                {'keyframe_meter_gap': 0.2},
                {'sc_dist_thres': 0.45}
            ],
            remappings=[
                ('/velodyne_cloud_registered_local', '/orora/cloud_local'),
                ('/aft_mapped_to_init', '/orora/odom')
            ]
        )
    ])
