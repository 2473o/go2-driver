from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # 声明 use_sim_time 参数
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        use_sim_time_arg,
        # static tf imu_link -> rslidar
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            # pitch 0
            arguments=["0.1710", "0.0", "0.0908", "0.0", "0.0", "0.0", "imu_link", "rslidar"],
            # 13
            # arguments=["0.1710", "0.0", "0.0908", "0.0", "0.2269", "0.0", "imu_link", "rslidar"],
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        # static tf imu_link -> utlidar # 垃圾雷达会丢点云
        # Node(
        #     package="tf2_ros",
        #     executable="static_transform_publisher",
        #     arguments=["0.315020", "0.0", "-0.089145", "0.0", "0.99134054", "0.0", "0.13131614", "imu_link", "utlidar_lidar"],
        #     parameters=[{'use_sim_time': use_sim_time}]
        # ),
        # static tf base_link -> imu_link
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["-0.02557", "0.0", "0.04232", "0.0", "0.0", "0.0", "base_link", "imu_link"],
            parameters=[{'use_sim_time': use_sim_time}]
        )
    ])
