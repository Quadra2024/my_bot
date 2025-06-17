#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        timer_period = 2.0  # 2 seconds delay to ensure AMCL is ready
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.pose_published = False

    def timer_callback(self):
        if self.pose_published:
            return

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()

        # ---- CHỈNH GIÁ TRỊ POSE TẠI ĐÂY ----
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.w = 1.0  # facing forward
        # Optional: set covariance
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.0685

        self.publisher_.publish(msg)
        self.get_logger().info('✅ Initial pose published')
        self.pose_published = True

def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    rclpy.spin_once(node, timeout_sec=5)  # Only run once
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

