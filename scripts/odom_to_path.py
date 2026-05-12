#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


class OdomToPath(Node):
    def __init__(self):
        super().__init__('odom_to_path')

        self.path = Path()
        self.path.header.frame_id = 'odom'

        self.odom_sub = self.create_subscription(
            Odometry, '/robot_odom', self.odom_cb, 10)

        self.path_pub = self.create_publisher(Path, '/robot_path', 10)

        self.get_logger().info('OdomToPath node started, publishing to /robot_path')

    def odom_cb(self, msg: Odometry):
        self.path.header.stamp = msg.header.stamp

        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.path.poses.append(pose)

        self.path_pub.publish(self.path)


def main():
    rclpy.init()
    node = OdomToPath()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()