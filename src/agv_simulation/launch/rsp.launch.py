import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 获取包路径
    pkg_share = get_package_share_directory('agv_simulation')
    xacro_file = os.path.join(pkg_share, 'description', 'robot.urdf.xacro')

    # 2. 定义 Launch 配置参数
    use_sim_time = LaunchConfiguration('use_sim_time')
    world_path = LaunchConfiguration('world')

    # 声明参数，允许在命令行通过 world:=... 修改
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true')
    
    declare_world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_share, 'worlds', 'simulation.world'),
        description='Full path to world file'
    )

    # 3. 设置环境变量 (针对 Jetson/虚拟机渲染问题的优化)
    # 如果 Gazebo 依然闪退，这两行能强制使用 CPU 渲染，保证稳定性
    set_render_env = SetEnvironmentVariable('SVGA_VGPU10', '0')
    set_driver_env = SetEnvironmentVariable('GALLIUM_DRIVER', 'llvmpipe')

    # 4. 机器人状态发布节点 (robot_state_publisher)
    robot_description_config = Command(['xacro ', xacro_file])
    params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # 5. 关节状态发布节点
    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 6. 启动 Gazebo Server (物理引擎)
    gzserver = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')
    ),
    launch_arguments={
        'world': world_path,
        'extra_gazebo_args': '--verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so'
    }.items()
)

    # 7. 启动 Gazebo Client (图形界面)
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py')
        )
    )

    # 8. 生成机器人实体 (Spawn Entity)
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description', 
            '-entity', 'my_agv', 
            '-x', '0', '-y', '-2', '-z', '0.1' # 设置在 y=-2 的位置，避免和坐标原点的方块重叠
        ],
    output='screen'
    )

    # 9. 启动 RViz2
    rviz_config_file = os.path.join(pkg_share, 'config', 'view_bot.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # 10. 启动 EKF 节点 (融合 IMU 和 里程计)
    ekf_config_path = os.path.join(pkg_share, 'config', 'ekf.yaml')
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path, {'use_sim_time': use_sim_time}]
    )

    # 返回 Launch 描述
    return LaunchDescription([
        set_render_env,
        set_driver_env,
        declare_use_sim_time,
        declare_world_arg,
        node_robot_state_publisher,
        node_joint_state_publisher,
        gzserver,
        gzclient,
        spawn_entity,
        rviz_node,
        ekf_node
    ])
