#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import numpy as np
import time

class SimPlotter(Node):
    def __init__(self):
        super().__init__('sim_plotter')

        # --- 設定 ---
        self.interval = 0.05  # 50ms周期
        self.noise_std = 0.05  # ノイズの標準偏差
        self.sim_drag = 0.15   # 慣性係数 (値が大きいほど応答が遅れる)

        # --- 通信設定 ---
        self.pub_target = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_odom = self.create_publisher(Odometry, 'odom', 10)
        self.sub_control = self.create_subscription(
            Twist, 
            'cmd_vel_out', 
            self.control_callback, 
            10
        )

        # --- 内部変数 (X, Y, Z それぞれ持つ) ---
        self.start_time = time.time()
        
        # 現在のシミュレーション上の速度
        self.sim_vel = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        
        # PIDから受け取った制御入力（初期値は0）
        self.ctrl_input = {'x': 0.0, 'y': 0.0, 'z': 0.0}

        # --- ログ用データ配列 ---
        self.logs = {
            'time': [],
            'target_x': [], 'odom_x': [], 'ctrl_x': [],
            'target_y': [], 'odom_y': [], 'ctrl_y': [],
            'target_z': [], 'odom_z': [], 'ctrl_z': []
        }

        self.timer = self.create_timer(self.interval, self.timer_callback)
        print("3軸同時シミュレーション開始... (Ctrl+C で終了してグラフを表示)")

    def control_callback(self, msg):
        # PIDノードからの出力を保存
        self.ctrl_input['x'] = msg.linear.x
        self.ctrl_input['y'] = msg.linear.y
        self.ctrl_input['z'] = msg.angular.z

    def timer_callback(self):
        elapsed = time.time() - self.start_time
        
        # --- 1. 目標速度の生成 (軸ごとに波形を変える) ---
        # X: 大きな正弦波 (周期 約6秒)
        tgt_x = 1.0 * np.sin(elapsed)
        # Y: 小さく速い正弦波 (周期 約3秒) -> 横移動
        tgt_y = 0.5 * np.cos(2.0 * elapsed)
        # Z: ゆっくりな正弦波 -> 旋回
        tgt_z = 0.8 * np.sin(0.5 * elapsed)

        # --- 2. 物理シミュレーション (一次遅れ系) ---
        # 各軸独立して計算
        for axis, target_val in zip(['x', 'y', 'z'], [self.ctrl_input['x'], self.ctrl_input['y'], self.ctrl_input['z']]):
            # 「指令値」に向かって「現在の速度」が近づく
            self.sim_vel[axis] += (target_val - self.sim_vel[axis]) * (1.0 - self.sim_drag)

        # --- 3. オドメトリ生成 (ノイズ付加) ---
        odom_x = self.sim_vel['x'] + np.random.normal(0, self.noise_std)
        odom_y = self.sim_vel['y'] + np.random.normal(0, self.noise_std)
        odom_z = self.sim_vel['z'] + np.random.normal(0, self.noise_std)

        # --- 4. ROS Publish ---
        # Target
        t_msg = Twist()
        t_msg.linear.x = tgt_x
        t_msg.linear.y = tgt_y
        t_msg.angular.z = tgt_z
        self.pub_target.publish(t_msg)

        # Odom
        o_msg = Odometry()
        o_msg.twist.twist.linear.x = odom_x
        o_msg.twist.twist.linear.y = odom_y
        o_msg.twist.twist.angular.z = odom_z
        self.pub_odom.publish(o_msg)

        # --- 5. ログ記録 ---
        self.logs['time'].append(elapsed)
        
        self.logs['target_x'].append(tgt_x)
        self.logs['odom_x'].append(odom_x)
        self.logs['ctrl_x'].append(self.ctrl_input['x'])

        self.logs['target_y'].append(tgt_y)
        self.logs['odom_y'].append(odom_y)
        self.logs['ctrl_y'].append(self.ctrl_input['y'])

        self.logs['target_z'].append(tgt_z)
        self.logs['odom_z'].append(odom_z)
        self.logs['ctrl_z'].append(self.ctrl_input['z'])

    def plot_graph(self):
        print("\nグラフを描画中...")
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 12), sharex=True)
        
        time_data = self.logs['time']

        # --- X軸のグラフ ---
        ax1.plot(time_data, self.logs['target_x'], label='Target X', color='blue', linewidth=2)
        ax1.plot(time_data, self.logs['odom_x'], label='Odom X', color='green', alpha=0.5)
        ax1.plot(time_data, self.logs['ctrl_x'], label='PID Out X', color='red', linestyle='--')
        ax1.set_title('Linear X (Forward/Backward)')
        ax1.set_ylabel('m/s')
        ax1.grid(True)
        ax1.legend(loc='upper right')

        # --- Y軸のグラフ ---
        ax2.plot(time_data, self.logs['target_y'], label='Target Y', color='blue', linewidth=2)
        ax2.plot(time_data, self.logs['odom_y'], label='Odom Y', color='green', alpha=0.5)
        ax2.plot(time_data, self.logs['ctrl_y'], label='PID Out Y', color='red', linestyle='--')
        ax2.set_title('Linear Y (Strafe)')
        ax2.set_ylabel('m/s')
        ax2.grid(True)
        ax2.legend(loc='upper right')

        # --- Z軸のグラフ ---
        ax3.plot(time_data, self.logs['target_z'], label='Target Z', color='blue', linewidth=2)
        ax3.plot(time_data, self.logs['odom_z'], label='Odom Z', color='green', alpha=0.5)
        ax3.plot(time_data, self.logs['ctrl_z'], label='PID Out Z', color='red', linestyle='--')
        ax3.set_title('Angular Z (Rotation)')
        ax3.set_ylabel('rad/s')
        ax3.set_xlabel('Time [s]')
        ax3.grid(True)
        ax3.legend(loc='upper right')

        plt.tight_layout()
        plt.show()

def main():
    rclpy.init()
    node = SimPlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.plot_graph()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
