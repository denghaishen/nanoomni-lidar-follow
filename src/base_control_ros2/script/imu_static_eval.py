#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
P2-3 IMU 静态噪声评估工具
============================
用途: 把机器人放在水平桌面上, 完全静止, 跑 30 秒,
      输出 gyro / accel 的均值与方差, 用于替换
      base_control_ros2.py 中 imu_msg 的经验协方差初值.

用法:
    1. 放好机器人, 保证完全静止 (桌面平整, 不要碰)
    2. 启动底盘:    ros2 launch robot_navigation_ros2 robot_lidar.launch.py
    3. 另一个终端: source ~/cartographer_ws/install/setup.bash
                   python3 ~/cartographer_ws/src/base_control_ros2/script/imu_static_eval.py
    4. 等 30 秒, 看输出统计

判读标准:
    - Gyro  std  应 < 0.01 rad/s    (好的 6/9 轴 IMU)
    - Accel std  应 < 0.1 m/s^2    (静止时除 z=g 外两轴接近 0)
    - Accel z 均值应接近 +9.8 (m/s^2), 偏差 > 0.5 说明加速度计未校准
    - Gyro 三轴均值应都 < 0.01 rad/s, 否则有零偏需要校准

输出 var (var = std^2) 可直接填入 imu_msg.angular_velocity_covariance / linear_acceleration_covariance.
"""

import rclpy
import statistics
from rclpy.node import Node
from sensor_msgs.msg import Imu
from rclpy.qos import qos_profile_sensor_data


SAMPLE_DURATION_SEC = 30.0


class ImuStaticEval(Node):
    def __init__(self):
        super().__init__('imu_static_eval')
        self.gyro = [[], [], []]
        self.accel = [[], [], []]
        self.quat = [[], [], [], []]
        self.start_time = self.get_clock().now()
        self.last_print = 0
        self.sub = self.create_subscription(
            Imu, 'imu', self.imu_cb, qos_profile_sensor_data
        )
        self.get_logger().info(
            f'Recording IMU for {SAMPLE_DURATION_SEC:.0f}s. Keep robot ABSOLUTELY still...'
        )

    def imu_cb(self, msg):
        self.gyro[0].append(msg.angular_velocity.x)
        self.gyro[1].append(msg.angular_velocity.y)
        self.gyro[2].append(msg.angular_velocity.z)
        self.accel[0].append(msg.linear_acceleration.x)
        self.accel[1].append(msg.linear_acceleration.y)
        self.accel[2].append(msg.linear_acceleration.z)
        self.quat[0].append(msg.orientation.w)
        self.quat[1].append(msg.orientation.x)
        self.quat[2].append(msg.orientation.y)
        self.quat[3].append(msg.orientation.z)

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if int(elapsed) != self.last_print and int(elapsed) % 5 == 0:
            self.last_print = int(elapsed)
            self.get_logger().info(
                f'  ... {int(elapsed)}s / {int(SAMPLE_DURATION_SEC)}s, samples={len(self.gyro[0])}'
            )
        if elapsed >= SAMPLE_DURATION_SEC:
            self.report_and_exit()

    def report_and_exit(self):
        n = len(self.gyro[0])
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        rate = n / elapsed if elapsed > 0 else 0

        print('\n' + '=' * 70)
        print(f' IMU Static Noise Report  (samples={n}, {elapsed:.1f}s, {rate:.1f} Hz)')
        print('=' * 70)

        # Gyro
        print('\n[ Angular Velocity ]  unit: rad/s')
        print(f'  {"axis":<6}{"mean":>14}{"std":>14}{"var":>14}')
        gv = [0.0, 0.0, 0.0]
        for i, name in enumerate(['x', 'y', 'z']):
            mean = statistics.mean(self.gyro[i])
            std = statistics.stdev(self.gyro[i])
            var = std * std
            gv[i] = var
            print(f'  {name:<6}{mean:>14.6f}{std:>14.6f}{var:>14.3e}')

        # Accel
        print('\n[ Linear Acceleration ]  unit: m/s^2')
        print(f'  {"axis":<6}{"mean":>14}{"std":>14}{"var":>14}')
        av = [0.0, 0.0, 0.0]
        for i, name in enumerate(['x', 'y', 'z']):
            mean = statistics.mean(self.accel[i])
            std = statistics.stdev(self.accel[i])
            var = std * std
            av[i] = var
            print(f'  {name:<6}{mean:>14.6f}{std:>14.6f}{var:>14.3e}')

        # Orientation Quaternion
        print('\n[ Orientation Quaternion ]')
        print(f'  {"axis":<6}{"mean":>14}{"std":>14}')
        for i, name in enumerate(['w', 'x', 'y', 'z']):
            if len(self.quat[i]) > 1:
                mean = statistics.mean(self.quat[i])
                std = statistics.stdev(self.quat[i])
                print(f'  {name:<6}{mean:>14.6f}{std:>14.6f}')

        # 判读
        print('\n' + '-' * 70)
        print(' 判读 (Quality Check)')
        print('-' * 70)
        gyro_max_std = max(statistics.stdev(self.gyro[i]) for i in range(3))
        accel_z_mean = statistics.mean(self.accel[2])
        gyro_z_bias = abs(statistics.mean(self.gyro[2]))

        if gyro_max_std < 0.005:
            print('  [GOOD] Gyro 噪声很低 (< 0.005 rad/s),  IMU 质量优秀')
        elif gyro_max_std < 0.02:
            print('  [OK]   Gyro 噪声中等 (< 0.02 rad/s),  可以使用')
        else:
            print('  [WARN] Gyro 噪声偏高 (> 0.02 rad/s),  use_imu_data 不建议开启')

        if abs(accel_z_mean - 9.8) < 0.3:
            print(f'  [GOOD] Accel Z 均值 {accel_z_mean:.3f} 接近 g=9.8,  加速度计已校准')
        elif abs(accel_z_mean) < 0.1 and abs(accel_z_mean - 9.8) > 0.3:
            print(f'  [WARN] Accel Z 均值 {accel_z_mean:.3f} 异常 (期望 ~9.8 m/s^2)')

        if gyro_z_bias > 0.01:
            print(f'  [WARN] Gyro Z 零偏 {gyro_z_bias:.4f} rad/s 偏大, 长时间静止会有方向漂移')
        else:
            print(f'  [GOOD] Gyro Z 零偏 {gyro_z_bias:.4f} rad/s, 已接近零偏校准')

        # 给出可直接复制的 covariance 数组
        print('\n' + '-' * 70)
        print(' 复制以下数组替换 base_control_ros2.py 中 imu_msg.*_covariance')
        print('-' * 70)
        print('\n        # angular_velocity_covariance (P2-3 实测)')
        print('        self.imu_msg.angular_velocity_covariance = [')
        print(f'            {gv[0]:.3e}, 0.0, 0.0,')
        print(f'            0.0, {gv[1]:.3e}, 0.0,')
        print(f'            0.0, 0.0, {gv[2]:.3e},')
        print('        ]')
        print('\n        # linear_acceleration_covariance (P2-3 实测)')
        print('        self.imu_msg.linear_acceleration_covariance = [')
        print(f'            {av[0]:.3e}, 0.0, 0.0,')
        print(f'            0.0, {av[1]:.3e}, 0.0,')
        print(f'            0.0, 0.0, {av[2]:.3e},')
        print('        ]')
        print()

        rclpy.shutdown()


def main():
    rclpy.init()
    node = ImuStaticEval()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
