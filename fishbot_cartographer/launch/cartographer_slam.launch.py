import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 配置文件路径
    cartographer_config_dir = os.path.join(
        get_package_share_directory('fishbot_cartographer'),
        'config'
    )
    configuration_basename = 'fishbot_slam.lua'

    # 是否启动 rviz2
    use_rviz = LaunchConfiguration('use_rviz', default='true')

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true',
                              description='是否启动 rviz2'),

        # Cartographer 建图节点
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            arguments=[
                '-configuration_directory', cartographer_config_dir,
                '-configuration_basename', configuration_basename,
            ],
            remappings=[
                ('scan', '/scan'),
                ('odom', '/odom'),
            ],
        ),

        # 占用栅格地图节点（把子图转成 /map 话题）
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{'resolution': 0.05}],
        ),

        # Rviz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(use_rviz),
        ),
    ])
