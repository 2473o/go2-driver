from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    sim_ = LaunchConfiguration('sim')
    use_cupcl_ = LaunchConfiguration('use_cupcl')
    queue_size_ = LaunchConfiguration('queue_size')
    qos_reliable_ = LaunchConfiguration('qos_reliable')
    max_cloud_age_ = LaunchConfiguration('max_cloud_age')
    max_points_per_scan_ = LaunchConfiguration('max_points_per_scan')
    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='false',
        description='Use simulation clock if true (maps to use_sim_time).',
    )
    use_cupcl_arg = DeclareLaunchArgument(
        'use_cupcl',
        default_value='true',
        description='Enable cuPCL when not running in simulation.',
    )
    queue_size_arg = DeclareLaunchArgument(
        'queue_size',
        default_value='1',
        description='PointCloud2 subscription queue size; keep this low to avoid latency.',
    )
    qos_reliable_arg = DeclareLaunchArgument(
        'qos_reliable',
        default_value='false',
        description='Use reliable QoS instead of best-effort sensor QoS.',
    )
    max_cloud_age_arg = DeclareLaunchArgument(
        'max_cloud_age',
        default_value='0.3',
        description='Drop cloud frames older than this many seconds; 0 disables stale-frame dropping.',
    )
    max_points_per_scan_arg = DeclareLaunchArgument(
        'max_points_per_scan',
        default_value='80000',
        description='Maximum points projected per LaserScan; 0 disables point subsampling.',
    )

    # pointcloud_accumulation_node for GO2 L1/L2 Lidar
    # 对于大体积实时点云，不建议使用
    # pcl_acc_node = Node(
    #     package='go2_perception', executable='cloud_accumulation',
    #     parameters=[{
    #         'input_topic': '/utlidar/cloud_base',
    #         'max_clouds': 30,
    #         'min_height': -0.3,
    #         'max_height': 1.0,
    #     }],
    #     remappings=[
    #         ('/utlidar/cloud_accumulated', '/trans_cloud')
    #         ],
    #     name='cloud_accumulation_node'
    # )

    pointcloud_to_laserscan =   Node(
            package='go2_driver', executable='pointcloud_to_laserscan_node',
            remappings=[
                ('cloud_in', '/rslidar_points'), 
                ('scan', '/scan')
                ],
            parameters=[{
                'target_frame': 'rslidar',
                'transform_tolerance': 0.1,
                'min_height': -0.3,
                'max_height': 1.0,
                'angle_min': -3.14,  
                'angle_max': 3.14,  
                'angle_increment': 0.00436,
                'scan_time': 0.1,
                'range_min': 0.00,          
                'range_max': 10.0,
                'use_inf': True,
                'inf_epsilon': 1.0,
                'queue_size': ParameterValue(queue_size_, value_type=int),
                'qos_reliable': ParameterValue(qos_reliable_, value_type=bool),
                'max_cloud_age': ParameterValue(max_cloud_age_, value_type=float),
                'max_points_per_scan': ParameterValue(max_points_per_scan_, value_type=int),
                'use_cupcl': ParameterValue(use_cupcl_, value_type=bool),
                'voxel_leaf_size': 0.02,
                'cupcl_retry_without_voxel': True,
                'use_sim_time': sim_
            }],
            name='pointcloud_to_laserscan_node'
        )


    return LaunchDescription([
        sim_arg,
        use_cupcl_arg,
        queue_size_arg,
        qos_reliable_arg,
        max_cloud_age_arg,
        max_points_per_scan_arg,
        # pcl_acc_node,
        pointcloud_to_laserscan
    ])
