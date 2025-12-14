#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import csv
import os
import datetime
import matplotlib.pyplot as plt
import numpy as np

class ResponseTestY(Node):
    def __init__(self):
        super().__init__('response_test_y')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        
        self.current_odom_twist = Twist()
        self.log_data = []
        
        # 結果保存用ディレクトリとファイル名の設定
        now_str = datetime.datetime.now().strftime('%m%d_%H%M')
        # デスクトップのresponse_testフォルダに保存
        self.result_dir = os.path.expanduser('~/Desktop/response_test')
        os.makedirs(self.result_dir, exist_ok=True)
        
        self.log_file = os.path.join(self.result_dir, f'response_test_y_{now_str}.csv')
        self.plot_file = os.path.join(self.result_dir, f'response_test_y_{now_str}.png')

        self.target_vel = 0.5  # m/s
        self.duration = 3.0    # seconds
        
        self.get_logger().info(f'Ready to run Y-axis step response test. (Numpy: {np.__version__})')
        self.run_test_sequence()

    def odom_callback(self, msg):
        self.current_odom_twist = msg.twist.twist

    def run_test_sequence(self):
        self.get_logger().info(f'Starting Test: Y={self.target_vel} m/s for {self.duration} sec')
        
        self.send_velocity(0.0, 0.0, 0.0)
        time.sleep(1.0)
        
        start_time = self.get_clock().now().nanoseconds / 1e9
        end_time = start_time + self.duration
        
        try:
            while rclpy.ok():
                now = self.get_clock().now().nanoseconds / 1e9
                elapsed = now - start_time
                
                if now < end_time:
                    cmd_y = self.target_vel
                    self.send_velocity(0.0, cmd_y, 0.0)
                else:
                    cmd_y = 0.0
                    self.send_velocity(0.0, 0.0, 0.0)
                    if now > end_time + 1.0:
                        break
                
                self.log_data.append([elapsed, cmd_y, self.current_odom_twist.linear.y])
                
                rclpy.spin_once(self, timeout_sec=0.01)
                time.sleep(0.01)
        except KeyboardInterrupt:
            pass
        finally:
            self.send_velocity(0.0, 0.0, 0.0)
            self.save_log()
            self.plot_data()

    def send_velocity(self, x, y, w):
        msg = Twist()
        msg.linear.x = float(x)
        msg.linear.y = float(y)
        msg.angular.z = float(w)
        self.publisher_.publish(msg)

    def save_log(self):
        with open(self.log_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['Time', 'Cmd_Y', 'Odom_Y'])
            writer.writerows(self.log_data)
        self.get_logger().info(f'Data saved to {os.path.abspath(self.log_file)}')

    def plot_data(self):
        # Numpy配列に変換
        data = np.array(self.log_data)
        if len(data) == 0:
            self.get_logger().warn('No data to plot.')
            return

        times = data[:, 0]
        cmds = data[:, 1]
        odoms = data[:, 2]

        plt.figure(figsize=(10, 6))
        plt.plot(times, cmds, label='Command Y (m/s)', linestyle='--')
        plt.plot(times, odoms, label='Odom Y (m/s)')
        plt.title('Step Response Test (Y-axis)')
        plt.xlabel('Time [s]')
        plt.ylabel('Velocity [m/s]')
        plt.legend()
        plt.grid(True)
        plt.savefig(self.plot_file)
        self.get_logger().info(f'Plot saved to {self.plot_file}')

def main(args=None):
    rclpy.init(args=args)
    node = ResponseTestY()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
