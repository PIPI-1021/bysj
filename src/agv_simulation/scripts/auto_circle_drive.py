#!/usr/bin/env python3
"""
优化版巡边建图节点 - 添加转弯补偿
从房间中心(0,0)出发 → 到达墙边 → 顺时针绕边一圈 → 返回中心
适用于 10m x 6m 的矩形围栏空间
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class CenterToEdgePatrol(Node):
    def __init__(self):
        super().__init__('center_to_edge_patrol')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 速度参数 
        self.linear_vel = 0.2    # 直线速度 (m/s)
        self.angular_vel = 0.5   # 旋转速度 (rad/s)
        self.corner_pause = 1.5  # 角落停顿时间 (SLAM稳定)
        
        # =转弯补偿系数 
        # 如果90°转不够，增大这个值；转过了就减小
        # 1.0 = 理论值 | 1.15 = 推荐起点 | 1.2~1.3 = 转不够时用
        self.turn_compensation = 1.2 
        
        # 路径规划 
        self.safe_margin = 0.5
        self.half_length = 5.0 - self.safe_margin   # 4.5m
        self.half_width = 3.0 - self.safe_margin    # 2.5m
        
        # 计算行走时间
        self.center_to_wall = self.half_length / self.linear_vel
        self.half_to_wall = self.half_width / self.linear_vel
        self.long_side_time = (self.half_length * 2) / self.linear_vel
        self.short_side_time = (self.half_width * 2) / self.linear_vel
        
        # 90度转弯时间（带补偿）
        base_turn_time = 1.57 / self.angular_vel
        self.turn_90_time = base_turn_time * self.turn_compensation
        
        self.get_logger().info(f'转弯补偿系数: {self.turn_compensation}')
        self.get_logger().info(f'90°转弯时间: {self.turn_90_time:.2f}s (理论: {base_turn_time:.2f}s)')
        
        time.sleep(2.0)
        self.run_patrol()

    def send_cmd(self, linear, angular, duration, description=""):
        if description:
            self.get_logger().info(f' {description} [{duration:.1f}s]')
        
        start_time = time.time()
        while rclpy.ok() and (time.time() - start_time) < duration:
            msg = Twist()
            msg.linear.x = linear
            msg.angular.z = angular
            self.publisher_.publish(msg)
            time.sleep(0.05)
        
        stop_msg = Twist()
        self.publisher_.publish(stop_msg)
        time.sleep(0.3)

    def turn_angle(self, angle_deg, direction='left'):
        """原地旋转指定角度（补偿后）"""
        angle_ratio = abs(angle_deg) / 90.0
        duration = self.turn_90_time * angle_ratio
        angular = self.angular_vel if direction == 'left' else -self.angular_vel
        self.send_cmd(0.0, angular, duration, f'原地转向 {angle_deg}° ({direction})')
        time.sleep(self.corner_pause)

    def run_patrol(self):
        self.get_logger().info(' 开始巡边建图任务')
        
        self.get_logger().info(' 阶段1: 从中心出发前往北墙边')
        self.send_cmd(self.linear_vel, 0.0, self.center_to_wall, 
                     f'直线前进到北墙边 ({self.half_length}m)')
        
        self.turn_angle(90, 'right')
        
        self.get_logger().info(' 阶段2: 开始顺时针绕边')
        
        self.send_cmd(self.linear_vel, 0.0, self.half_to_wall, 
                     f'沿北墙向东 ({self.half_width}m)')
        self.turn_angle(90, 'right')
        
        self.send_cmd(self.linear_vel, 0.0, self.long_side_time, 
                     f'沿东墙向南 ({self.half_length*2}m)')
        self.turn_angle(90, 'right')
        
        self.send_cmd(self.linear_vel, 0.0, self.short_side_time, 
                     f'沿南墙向西 ({self.half_width*2}m)')
        self.turn_angle(90, 'right')
        
        self.send_cmd(self.linear_vel, 0.0, self.long_side_time, 
                     f'沿西墙向北 ({self.half_length*2}m)')
        """
        self.get_logger().info(' 阶段3: 返回中心')
        self.turn_angle(180, 'left')
        self.send_cmd(self.linear_vel, 0.0, self.center_to_wall, 
                     f'返回中心 ({self.half_width}m)')
        
        self.turn_angle(90, 'left')
        """
        self.send_cmd(0.0, 0.0, 2.0, '任务完成，停止')
        self.get_logger().info('巡边建图任务全部完成！')


def main(args=None):
    rclpy.init(args=args)
    node = CenterToEdgePatrol()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
