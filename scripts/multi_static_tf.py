#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
import math

def euler_to_quaternion(yaw, pitch, roll):
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return [qx, qy, qz, qw]

class MultiStaticTfPublisher(Node):
    def __init__(self):
        super().__init__('multi_static_tf_publisher')
        
        # 静态TF必须使用 Transient Local 的 QoS (类似 ROS1 的 latch=True)
        qos_profile = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        
        # 直接创建一个 Publisher 发布到 /tf_static
        self.publisher = self.create_publisher(TFMessage, '/tf_static', qos_profile)
        
        transforms = []
        
        # 定义一个辅助函数快速添加 TF
        def add_tf(x, y, z, yaw, pitch, roll, parent, child):
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = parent
            t.child_frame_id = child
            t.transform.translation.x = float(x)
            t.transform.translation.y = float(y)
            t.transform.translation.z = float(z)
            q = euler_to_quaternion(float(yaw), float(pitch), float(roll))
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            transforms.append(t)

        # 批量添加你需要的 TF (x, y, z, yaw, pitch, roll, parent, child)
        add_tf(0.1710, 0.0, 0.0908, 0.0, 0.0, 0.0, "imu_link", "rslidar")
        add_tf(-0.02557, 0.0, 0.04232, 0.0, 0.0, 0.0, "base_link", "imu_link")
        add_tf(0.34463, 0.01128, 0.09685, 0.0, 0.0, 0.0, "base_link", "realsense_cam_link")
        # add_tf(0.28945, 0.0, -0.046825, 0.0, 3.141592653589793, 0.0, "base_link", "utlidar_lidar")

        # 将所有的 TransformStamped 包装进一个 TFMessage 中一次性发布
        tf_msg = TFMessage()
        tf_msg.transforms = transforms
        
        # 发布消息
        self.publisher.publish(tf_msg)
        self.get_logger().info('Successfully published all static transforms.')

def main():
    rclpy.init()
    node = MultiStaticTfPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
