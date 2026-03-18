import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. 获取包路径
    pkg_path = get_package_share_directory('agv_simulation')
    
    # 2. 声明参数（方便在终端临时修改）
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')

    declare_slam_params_file_argument = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(pkg_path, 'config', 'mapper_params_online_async.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

    # 3. 启动 slam_toolbox 节点
    # 注意：它会发布 map -> odom 的 TF 变换
    start_slam_toolbox_node = Node(
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen'
    )

    # 4. 返回启动描述
    return LaunchDescription([
        declare_use_sim_time_argument,
        declare_slam_params_file_argument,
        start_slam_toolbox_node
    ])
