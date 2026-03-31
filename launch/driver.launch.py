from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import math
import os

def generate_launch_description():
    lidar_cloud_topic = DeclareLaunchArgument(
        "lidar_cloud_topic",
        default_value="/rslidar_points",
        description="输入点云话题，默认保持 pointcloud_to_laserscan 原生话题名"
    )
    laser_scan_topic = DeclareLaunchArgument(
        "laser_scan_topic",
        default_value="/rslidar_scan",
        description="输出激光扫描话题"
    )
    laser_target_frame = DeclareLaunchArgument(
        "laser_target_frame",
        default_value="rslidar",
        description="点云转换后的目标坐标系"
    )

    # lowstate driver, pub imu and leg sensor
    lowstate_driver =   Node(
            package="go2_driver",
            executable="lowstate_driver",
            parameters=[{'imu_enable': False,
                         'leg_sensor_enable': True,}],
    )

    # head camera
    camera =  Node(
            package="go2_driver",
            executable="head_camera",
            parameters=[{
                'pub_camera_raw_enable': True,
                'pub_camera_compressed_enable': True,
                "pub_camera_topic": "/head_camera/image_raw",
                'network_interface': 'eth0',
                'gst_pipeline': (
                    'udpsrc address=230.1.1.1 port=1720 multicast-iface=eth0 ! '
                    'application/x-rtp,media=video,encoding-name=H264 ! rtph264depay ! '
                    'h264parse ! nvv4l2decoder enable-max-performance=1 ! '
                    'nvvidconv output-buffers=1 ! '
                    'video/x-raw,format=BGRx,width=1280,height=720 ! '
                    'videoconvert ! video/x-raw,format=BGR ! appsink drop=1 sync=false'
                )
            }],
    )

    sport_driver = Node(
            package="go2_driver",
            executable="sport_driver"
        )

    machine = os.uname().machine.lower()
    is_aarch64 = machine in ("aarch64", "arm64")
    max_height_val = 1.81
    angle_increment_val = math.pi / 180.0

    if is_aarch64:
        queue_size_val = 2.0
        min_height_val = -0.12
        range_max_val = 30.0
    else:
        queue_size_val = 4.0
        min_height_val = -0.15
        range_max_val = 30.0

    laser_scan = Node(
            package="pointcloud_to_laserscan",
            executable="pointcloud_to_laserscan_node",
            name="pointcloud_to_laserscan",
            remappings=[
                ("cloud_in", LaunchConfiguration("lidar_cloud_topic")),
                ("scan", LaunchConfiguration("laser_scan_topic")),
            ],
            parameters=[{
                "target_frame": LaunchConfiguration("laser_target_frame"),
                "transform_tolerance": 0.01,
                "min_height": min_height_val,
                "max_height": max_height_val,
                "angle_min": -math.pi,
                "angle_max": math.pi,
                "angle_increment": angle_increment_val,
                "scan_time": 1.0 / 30.0,
                "range_min": 0.1,
                "range_max": range_max_val,
                "use_inf": True,
                "queue_size": queue_size_val,
            }]
    )
    

    return LaunchDescription([
        lidar_cloud_topic,
        laser_scan_topic,
        laser_target_frame,
        # camera,
        lowstate_driver,
        laser_scan,
        sport_driver,
])
