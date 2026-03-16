import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # 1. 点云转激光节点配置
    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', '/points_raw'),
            ('scan', '/scan'),
        ],
        parameters=[{
            'target_frame': 'lidar_link', # 需与你点云的 frame_id 一致
            'transform_tolerance': 0.01,
            'min_height': -1.0,           # 适当放宽高度范围确保切到数据
            'max_height': 1.0,
            'angle_min': -3.1415,
            'angle_max': 3.1415,
            'angle_increment': 0.0087,   # 约 0.5 度
            'scan_time': 0.1,
            'range_min': 0.3,
            'range_max': 50.0,
            'use_inf': True,
            'inf_epsilon': 1.0
        }],
        output='screen'
    )

    # 2. SLAM Toolbox 节点（可选，建议也放进来一键启动）
    # 这里也可以添加 slam_toolbox 的启动逻辑

    return LaunchDescription([
        pointcloud_to_laserscan_node,
    ])
