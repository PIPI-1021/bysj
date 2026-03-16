import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Imu
import message_filters

class FusionNode(Node):
    def __init__(self):
        super().__init__('perception_fusion_node')

        # 创建订阅器
        self.lidar_sub = message_filters.Subscriber(self, PointCloud2, '/points_raw')
        self.imu_sub = message_filters.Subscriber(self, Imu, '/imu_data')

        # 时间同步器：允许 0.05 秒的时间差
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.lidar_sub, self.imu_sub], 10, 0.05
        )
        self.ts.registerCallback(self.fusion_callback)

    def fusion_callback(self, lidar_msg, imu_msg):
        # 这里的两个数据包时间戳是“对齐”的
        self.get_logger().info('收到同步后的雷达与IMU数据！')
        
        # 融合逻辑示例：
        # 1. 获取 IMU 的加速度判断当前是否颠簸
        # 2. 如果颠簸严重，增加点云过滤的阈值，防止误报地面为障碍物
        accel_z = imu_msg.linear_acceleration.z
        if abs(accel_z - 9.8) > 1.0:
            self.get_logger().warn('检测到颠簸，动态调整过滤参数')

def main():
    rclpy.init()
    node = FusionNode()
    rclpy.spin(node)
