import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('agv_simulation'),
        'config',
        'ekf.yaml'
    )

    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[config_file,{'use_sim_time': True}],
            # 如果需要重映射话题，可以在这里添加 remappings
            # remappings=[('/odometry/filtered', '/my_filtered_odom')]
        )
    ])
