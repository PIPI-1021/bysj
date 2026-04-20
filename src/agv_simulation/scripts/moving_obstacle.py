import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class MovingObstacle(Node):
    def __init__(self):
        super().__init__('moving_obstacle_node')
        # 创建发布者，控制障碍物移动
        self.publisher_ = self.create_publisher(Twist, '/obstacle/cmd_vel', 10)
        self.timer_period = 0.1  # 10Hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        self.speed = 0.5        # 移动速度 (m/s)
        self.duration = 4.0     # 单向移动时间 (秒)
        self.start_time = time.time()
        self.move_forward = True

    def timer_callback(self):
        msg = Twist()
        current_time = time.time()
        
        # 逻辑：每隔 duration 秒切换一次方向
        if current_time - self.start_time > self.duration:
            self.move_forward = not self.move_forward
            self.start_time = current_time
            self.get_logger().info(f'切换方向: {"前进" if self.move_forward else "后退"}')

        if self.move_forward:
            msg.linear.x = self.speed
        else:
            msg.linear.x = -self.speed
            
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MovingObstacle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # 停止时给一个 0 速度，防止障碍物一直滑行
        node.publisher_.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
