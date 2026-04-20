import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 获取包路径
    pkg_share = FindPackageShare('agv_simulation')
    launch_dir = PathJoinSubstitution([pkg_share, 'launch'])
    """
    # 获取 Python 脚本的路径 (指向 src 下的源文件路径，方便调试)
    # 如果你已经完成了 colcon build 并安装了，可以使用更标准的 Node 启动方式
    obstacle_script_path = PathJoinSubstitution([
        pkg_share, 'scripts', 'moving_obstacle.py'
    ])
    """    
    # 1. 仿真启动 (sim) - 立即启动
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([launch_dir, 'sim_launch.py'])
        ]),
    )
    """
    # 2. 动态障碍物控制脚本 - 建议延迟 3-5 秒启动，确保 Gazebo 已经完全加载模型
    # 使用 ExecuteProcess 直接运行 python3 脚本
    moving_obstacle_node = ExecuteProcess(
        cmd=['python3', obstacle_script_path],
        output='screen',
    )
    obstacle_delayed = TimerAction(
        period=4.0,
        actions=[moving_obstacle_node]
    )
    """   
    # 2. EKF滤波 (ekf) - 延迟2秒启动，等待仿真就绪
    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([launch_dir, 'ekf_launch.py'])
        ]),
    )
    ekf_delayed = TimerAction(
        period=2.0,
        actions=[ekf_launch]
    )

    # 3. SLAM建图 (slam) - 延迟5秒启动，等待EKF稳定
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([launch_dir, 'slam.launch.py'])
        ]),
    )
    slam_delayed = TimerAction(
        period=5.0,
        actions=[slam_launch]
    )
    """
    # 4. 巡边建图节点 - 延迟8秒启动，等待SLAM初始化完成
    # 巡边节点在scripts目录下，通过ros2 run启动
    patrol_node = ExecuteProcess(
        cmd=['ros2', 'run', 'agv_simulation', 'auto_circle_drive.py'],
        output='screen',
    )
    patrol_delayed = TimerAction(
        period=8.0,
        actions=[patrol_node]
    )
    """
    return LaunchDescription([
        sim_launch,      # T+0s: 立即启动
        ekf_delayed,     # T+2s: EKF定位
        # obstacle_delayed,# T+4s: 启动动态障碍物 
        slam_delayed,    # T+5s: SLAM建图
        # patrol_delayed,  # T+8s: 自动巡边建图
    ])
