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

    use_rviz = DeclareLaunchArgument(
        name="use_rviz",
        default_value="false"
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    # lowstate driver, pub imu and leg sensor
    lowstate_driver =   Node(
            package="go2_driver",
            executable="lowstate_driver",
            parameters=[{'use_sim_time': use_sim_time,
                         'imu_enable': False,
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

    # cmd_vel to sport request
    cmd_vel = Node(
            package="go2_driver",
            executable="sportstate_cmd_vel",
            parameters=[{'use_sim_time': use_sim_time}],
        )
    

    return LaunchDescription([
        use_rviz,
        use_sim_time_arg,

        # camera,
        lowstate_driver,
        cmd_vel,

])
