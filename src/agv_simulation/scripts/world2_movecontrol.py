#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

def main():
    rclpy.init()
    # 创建节点
    node = rclpy.create_node('obstacle_behavior_controller')
    
    # 重点：这里的 Topic 必须对应你 XML 里的 /obstacle 命名空间
    pub = node.create_publisher(Twist, '/obstacle/cmd_vel', 10)

    move_cmd = Twist()
    stop_cmd = Twist()
    
    speed = 0.5       # 移动速度 m/s
    move_duration = 5 # 移动时间（根据走廊宽度调整）
    wait_duration = 20.0 # 原地停留时间

    print("--- 障碍物控制脚本已启动 ---")
    print(f"控制目标话题: /obstacle/cmd_vel")

    try:
        while rclpy.ok():
            # 1. 从左向右移动 (沿 Y 轴正方向)
            print(">>> 状态：向右移动")
            move_cmd.linear.y = speed
            pub.publish(move_cmd)
            time.sleep(move_duration)

            # 2. 原地停 5s
            print("||| 状态：停止等待 5s")
            pub.publish(stop_cmd)
            time.sleep(wait_duration)

            # 3. 从右向左移动 (沿 Y 轴负方向)
            print("<<< 状态：向左移动")
            move_cmd.linear.y = -speed
            pub.publish(move_cmd)
            time.sleep(move_duration)

            # 4. 原地停 5s
            print("||| 状态：停止等待 5s")
            pub.publish(stop_cmd)
            time.sleep(wait_duration)

    except KeyboardInterrupt:
        # 退出时发布停止指令，防止方块飞出去
        pub.publish(stop_cmd)
        print("\n脚本已手动停止")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
