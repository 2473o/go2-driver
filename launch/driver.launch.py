from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_prefix

import math
import os
import sys

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
            parameters=[{'imu_enable': True,
                         'leg_sensor_enable': False,
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

    pkg_lib_dir = os.path.join(get_package_prefix('go2_driver'), 'lib', 'go2_driver')

    tf_static = ExecuteProcess(
        cmd=[sys.executable, '-u', os.path.join(pkg_lib_dir, 'multi_static_tf.py'),
             '--ros-args', '-p', ['use_sim_time:=', sim_]],
        output='screen',
        name='multi_static_tf',
    )

    odom_to_path = ExecuteProcess(
        cmd=[sys.executable, '-u', os.path.join(pkg_lib_dir, 'odom_to_path.py'),
             '--ros-args', '-p', ['use_sim_time:=', sim_]],
        output='screen',
        name='odom_to_path',
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
