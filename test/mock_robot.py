#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time

class MockRobot(Node):
    def __init__(self):
        super().__init__('mock_robot')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        self.publisher_ = self.create_publisher(Odometry, '/odom', 10)
        
        self.factor = 0.8
        self.get_logger().info(f'Mock Robot Started. Publishing /odom = /cmd_vel * {self.factor}')

    def cmd_vel_callback(self, msg):
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        
        # Simulate response with factor
        odom_msg.twist.twist.linear.x = msg.linear.x * self.factor
        odom_msg.twist.twist.linear.y = msg.linear.y * self.factor
        odom_msg.twist.twist.angular.z = msg.angular.z * self.factor
        
        self.publisher_.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MockRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
