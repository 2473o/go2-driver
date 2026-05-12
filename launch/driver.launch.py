from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition

import math
import os

def generate_launch_description():
    sim_ = LaunchConfiguration('sim')
    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='false',
        description='Use simulation clock if true (maps to use_sim_time).',
    )

    odom_ = LaunchConfiguration('odom')
    odom_arg = DeclareLaunchArgument(
        'odom',
        default_value='false',
        description='Use simulation clock if true (maps to use_sim_time).',
    )


    # lowstate driver, pub imu and leg sensor
    lowstate_driver =   Node(
            package="go2_driver",
            executable="lowstate_driver",
            parameters=[{'imu_enable': False,
                         'leg_sensor_enable': True,
                         'use_sim_time': sim_,}],
    )
    # head camera
    camera =  Node(
            package="go2_driver",
            executable="head_camera",
            # condition=IfCondition(EqualsSubstitution(sim_, 'false')),
            condition=IfCondition(
                PythonExpression([
                    "'", sim_, "'", " == 'false'"
                    ])
            ),
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
            executable="sport_driver",
            parameters=[{'use_sim_time': sim_, 'odom': odom_}],
    )

    tf_static = Node(
        package="go2_driver",
        executable="multi_static_tf.py",
        parameters=[{'use_sim_time': sim_}],
    )

    odom_to_path = Node(
        package="go2_driver",
        executable="odom_to_path.py",
        parameters=[{'use_sim_time': sim_}],
        # condition=IfCondition(EqualsSubstitution(odom_, 'true')), # foxy 不存在
        condition=IfCondition(
            PythonExpression([
                 "'", odom_, "'", " == 'true'"
            ])
        )
    )

    return LaunchDescription([
        sim_arg,
        odom_arg,
        # camera,
        lowstate_driver,
        sport_driver,
        tf_static,
        odom_to_path,
])
