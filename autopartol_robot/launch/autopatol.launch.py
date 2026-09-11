import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # 获取默认路径
    autopartol_robot_path = get_package_share_directory('autopartol_robot')
    partol_config_path = autopartol_robot_path + '/config/partol_config.yaml'

    # 声明 launch 参数：是否启动真机相机节点
    declare_use_camera = DeclareLaunchArgument(
        'use_camera',
        default_value='true',
        description='Whether to start the fishbot_camera node'
    )
    use_camera = LaunchConfiguration('use_camera')

    action_partol_node = launch_ros.actions.Node(
        package='autopartol_robot',
        executable='partol_node',
        output='screen',
        parameters=[partol_config_path]
    )

    # 语音播报节点
    action_speaker_node = launch_ros.actions.Node(
        package='autopartol_robot',
        executable='speaker',
        output='screen'
    )

    # 真机相机节点（可选）
    action_camera_node = launch_ros.actions.Node(
        package='fishbot_camera',
        executable='camera_driver',
        name='fishbot_camera',
        output='screen',
        condition=launch.conditions.IfCondition(use_camera)
    )

    return launch.LaunchDescription([
        declare_use_camera,
        action_partol_node,
        action_speaker_node,
        action_camera_node,
    ])
