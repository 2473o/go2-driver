from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # pointcloud_accumulation_node for GO2 L1/L2 Lidar
        # 对于大体积实时点云，不建议使用
        # Node(
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
        # ),
        Node(
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
                'use_cupcl': True,
                'voxel_leaf_size': 0.02,
                'cupcl_retry_without_voxel': True
            }],
            name='pointcloud_to_laserscan_node'
        )
    ])
