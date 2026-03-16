#3D点云降维,启动异步 SLAM
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_share = get_package_share_directory('agv_simulation')
    
    # 1. 配置文件路径
    slam_config = os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml')

    # 2. 3D点云转2D激光雷达节点 (关键：匹配16线雷达)
    scan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        parameters=[{
            'target_frame': 'base_link', # 投影到机器人中心 [cite: 13]
            'transform_tolerance': 0.05,
            'min_height': 0.1,   # 略高于地面，过滤地面点云 [cite: 6]
            'max_height': 1.0,   # 包含行人上半身高度，用于动态避障识别 [cite: 6]
            'range_min': 0.3,    # 匹配xacro中的min_range
            'range_max': 12.0,   # 匹配xacro中的max_range
            'use_sim_time': True,
            'use_inf': True,     
            'inf_epsilon': 1.0
        }],
        remappings=[
            ('cloud_in', '/points_raw'), # 订阅xacro中定义的雷达话题
            ('scan', '/scan')            # 发布给SLAM和避障模块
        ]
    )

    # 3. 启动 Slam Toolbox 异步建图 [cite: 5, 12]
    slam_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': slam_config, 
            'use_sim_time': 'True'
        }.items()
    )

    return LaunchDescription([
        scan_node, 
        slam_node
    ])
