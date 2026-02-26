from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
from launch.conditions import IfCondition

def generate_launch_description():

    # go2_desc_pkg = get_package_share_directory("go2_description")
    # go2_driver_pkg = get_package_share_directory("go2_driver")

    #为 rviz2 启动添加开关
    use_rviz = DeclareLaunchArgument(
        name="use_rviz",
        default_value="false"      # 调试时使用，运行slam时关闭，在主程序中打开对应的rviz2
    )

    # 声明 use_sim_time 参数
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    # sync_utlidar_cloud_time = DeclareLaunchArgument(
    #     name="sync_utlidar_cloud_time",
    #     default_value="true"
    # )

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        use_rviz,
        use_sim_time_arg,
        # sync_utlidar_cloud_time,
        # 机器人模型可视化
        # IncludeLaunchDescription(
        #     launch_description_source = PythonLaunchDescriptionSource(
        #         launch_file_path=os.path.join(go2_desc_pkg, "launch", "display.launch.py")
        #     ),
        #     launch_arguments=[("use_joint_state_publisher", "false")]   #关节状态有driver节点发布，不需要使用默认
        # ),
        # 包含 rviz2
        # Node(
        #     package="rviz2",
        #     executable="rviz2",
        #     arguments=["-d", os.path.join(go2_driver_pkg, "rviz", "display.rviz")],
        #     condition=IfCondition(LaunchConfiguration("use_rviz")),
        #     parameters=[{'use_sim_time': use_sim_time}]
        # ),
    
        # 速度消息桥接
        # Node(
            # package="go2_twist_bridge",
            # executable="twist_bridge"
        # ),

        # 里程计消息发布、广播里程计坐标、发布关节状态信息
        # Node(
        #     package="go2_driver",
        #     executable="driver",
        #     parameters=[os.path.join(go2_driver_pkg, "params", "driver.yaml")]
        # ),
        # Node(
        #     package="go2_driver",
        #     executable="odom_tf_node",
        #     parameters=[{'publish_odom': False}, {'publish_tf': False}, {'use_sim_time': use_sim_time}],
        # ),
        # Node(
        #     package="go2_driver",
        #     executable="joint_state_pub_node",
        #     parameters=[{'use_sim_time': use_sim_time}],
        # ),
        # imu
        Node(
            package="go2_driver",
            executable="lowstate_driver", # pub imu and leg sensor
            parameters=[{'use_sim_time': use_sim_time,
                         'leg_sensor_enable': True,}],
        ),
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
        # static tf imu_link -> utlidar
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0.315020", "0.0", "-0.089145", "0.0", "0.99134054", "0.0", "0.13131614", "imu_link", "utlidar_lidar"],
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        # static tf base_link -> imu_link
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["-0.02557", "0.0", "0.04232", "0.0", "0.0", "0.0", "base_link", "imu_link"],
            parameters=[{'use_sim_time': use_sim_time}]
        )
    ])
