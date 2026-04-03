import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 获取包路径
    pkg_share = FindPackageShare('agv_simulation')
    launch_dir = PathJoinSubstitution([pkg_share, 'launch'])

    # 1. 仿真启动 (sim) - 立即启动
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([launch_dir, 'sim_launch.py'])
        ]),
    )

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
        slam_delayed,    # T+5s: SLAM建图
        # patrol_delayed,  # T+8s: 自动巡边建图
    ])
