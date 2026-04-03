import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    # 1. 获取包路径
    pkg_share = get_package_share_directory('agv_simulation')
    
    # 2. 设定默认路径参数
    # 确保你的 yaml 文件路径正确
    default_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    default_map_yaml = os.path.join(pkg_share, 'maps', 'my_map.yaml')

    # 3. 定义 Launch 配置项
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    params_file = LaunchConfiguration('params_file', default=default_params_file)
    map_yaml_file = LaunchConfiguration('map', default=default_map_yaml)

    # 4. 定义要启动的导航节点
    nav_nodes = [
        'controller_server',
        'planner_server',
        'recoveries_server', # Humble 建议包含恢复行为服务器
        'bt_navigator',
        'waypoint_follower'
    ]

    # 5. 节点配置
    nodes_inst = [
        # Map Server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'yaml_filename': map_yaml_file}]
        ),
        # AMCL 定位
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[params_file]
        ),
        # 控制器 (Local Planner)
        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[params_file]
        ),
        # 规划器 (Global Planner)
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_file]
        ),
        # 行为树导航
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[params_file]
        ),
        # 行为服务器 (备份、旋转等恢复动作)
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='recoveries_server',
            parameters=[params_file],
            output='screen'
        ),
        # 生命周期管理器 (核心：负责按顺序启动上述所有节点)
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['map_server', 'amcl', 'controller_server', 
                               'planner_server', 'recoveries_server', 'bt_navigator']
            }]
        )
    ]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map', default_value=default_map_yaml),
        DeclareLaunchArgument('params_file', default_value=default_params_file),
        *nodes_inst
    ])
