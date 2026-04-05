import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    # 1. 获取功能包的路径
    pkg_share = get_package_share_directory('agv_simulation')

    # 2. 设定文件路径
    default_map_path = os.path.join(pkg_share, 'maps', 'my_map.yaml')
    default_params_path = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    # 3. 定义 Launch 参数（方便在命令行临时修改）
    map_yaml_file = LaunchConfiguration('map', default=default_map_path)
    params_file = LaunchConfiguration('params_file', default=default_params_path)
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # 4. 定义需要管理的节点名称列表（用于生命周期管理器）
    lifecycle_nodes = [
        'map_server', 
        'amcl', 
        'planner_server', 
        'controller_server', 
        'behavior_server', 
        'bt_navigator'
    ]

    # 5. 准备节点配置
    return LaunchDescription([
        # 声明参数
        DeclareLaunchArgument('map', default_value=default_map_path, description='Full path to map yaml file to load'),
        DeclareLaunchArgument('params_file', default_value=default_params_path, description='Full path to the ROS2 parameters file to use'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true'),

        # Map Server - 加载地图
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_yaml_file},
                        {'use_sim_time': use_sim_time}]
        ),

        # AMCL - 定位
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}]
        ),

        # Planner Server - 全局规划 (A*)
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}]
        ),

        # Controller Server - 局部规划 (DWA/DWB)
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            # 重映射 cmd_vel，如果你的机器人监听其他话题（如 /model/agv/cmd_vel）
            # remappings=[('/cmd_vel', '/cmd_vel')] 
        ),

        # Behavior Server - 恢复行为
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}]
        ),

        # BT Navigator - 行为树导航
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}]
        ),

        # Lifecycle Manager - 控制以上节点的启动顺序和状态切换
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'node_names': lifecycle_nodes}]
        ),
    ])
