import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    package_name = 'agv_simulation'
    pkg_path = get_package_share_directory(package_name)
    
    # 1. 解析 Xacro
    xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_desc = robot_description_config.toxml()

    # 2. robot_state_publisher 节点
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

    # 3. Gazebo 启动文件
    world_path = os.path.join(pkg_path, 'worlds', 'world1.world')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world_path}.items() 
    )
    
    # 4. Spawn Entity 节点
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', 
            '-entity', 'my_agv',
            '-x', '1.0',    # 设置初始 X 坐标
            '-y', '1.0',    # 设置初始 Y 坐标
            '-z', '0.1',    # 设置高度，防止轮子陷在地里
            '-Y', '0.0'     # 设置初始朝向（弧度，0.0 表示朝向 X 轴正方向）
        ],
        output='screen'
    )


    # 5. RViz2 节点
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}] # 同步仿真时间
    )
    
 
    return LaunchDescription([
        node_robot_state_publisher,
        gazebo,
        spawn_entity,
        node_rviz
    ])
