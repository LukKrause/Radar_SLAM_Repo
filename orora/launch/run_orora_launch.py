from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('seq_dir', default_value='/home/lukas/Downloads/KAIST_Dataset/KAIST03'),
        DeclareLaunchArgument('do_slam', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),

        ComposableNodeContainer(
            name='orora_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='orora',
                    plugin='orora::OroraOdomNode',
                    name='orora_odom',
                    parameters=[
                        {'seq_dir': LaunchConfiguration('seq_dir')},
                        {'do_slam': LaunchConfiguration('do_slam')},
                        #############################
                        # from ROS1 Parameter.yaml
                        #############################
                        {'use_doppler_compensation': True},
                        {'use_deskewing': False},                   # It's weird, but deskewing rather decreases pose estimation in MulRan dataset
                        {'use_voxelization': True},
                        {'voxel_size': 0.6},                        # [m]    
                        {'noise_bound': 0.75},                      # dependent on `voxel_size`. `noise_bound` is set to range between 1.0 * `voxel_size` to 2.0 * `voxel_size`
                        {'noise_bound_radial': 0.1},                # [m]
                        {'noise_bound_tangential': 9.0},            # [Deg]
                        {'deskewing_target': 'rot'},
                        #############################
                        # Not included in paper
                        # Originally, as the number of feature pairs increases, computational cost of PMC also dramatically increase.
                        # We observed that stopped motion results in too many feature matching correspondences.
                        # Therefore, we empirically set `num_feat_thr_for_stop_motion` to check whether a robot is stopped or not
                        #############################
                        {'stop_motion': {
                            'check_stop_motion': True,
                            'num_feat_thr_for_stop_motion': 700     # Note that it depends on the `voxel_size`
                        }},
                        #############################
                        # Less important parameters
                        #############################
                        # `noise_bound_coeff` plays a role as an uncertainty multiplier and is used when estimating COTE.
                        # I.e. final noise bound is set to `noise_bound` * `noise_bound_coeff`
                        {'noise_bound_coeff': 1.0},
                        {'rotation': {
                            # Num. max iter for the rotation estimation.
                            # Usually, rotation estimation converges within < 20 iterations
                            'num_max_iter': 100,
                            # Control the magnitue of the increase in non-linearity. In case of TLS, usually `gnc_factor` is set to 1.4
                            # The larger the value, the steeper the increase in nonlinearity.
                            'gnc_factor': 1.4,
                            # The cost threshold is compared with the difference between costs of consecutive iterations.
                            # Once the diff. of cost < `rot_cost_diff_thr`, then the optimization is finished.
                            'rot_cost_diff_thr': 0.000001
                        }},
                        {'algorithm': 'ORORA'},     # if parameter needed
                        {'dataset': 'mulran'},      # if parameter needed
                        {'viz_extraction': False},  # if parameter needed
                        {'viz_matching': False},    # if parameter needed
                        {'frame_rate': 4},          # if parameter needed
                        {'stop_each_frame': False}  # if parameter needed
                    ]
                )
            ],
            output='screen',
        ),

        GroupAction(
            condition=LaunchConfiguration('rviz'),
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='orora_rviz',
                    arguments=['-d', '/home/lukas/ros2_slam_ws/src/navtech-radar-slam/outlier-robust-radar-odometry/rviz/orora_viz.rviz'],
                    output='screen',
                )
            ]
        )
    ])
