import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # ==========================================
    # 1. 定位默认的 YAML 参数文件路径
    # ==========================================
    pkg_share_dir = get_package_share_directory('hikvision_driver')
    default_config_path = os.path.join(pkg_share_dir, 'config', 'camera_params.yaml')

    # ==========================================
    # 2. 声明 Launch 参数 (允许从命令行覆盖 YAML 配置)
    # ==========================================
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_path,
        description='Absolute path to the camera parameter YAML file.'
    )

    # 依然保留 camera_name 参数，用于动态决定节点的命名空间 (Namespace)
    camera_name_arg = DeclareLaunchArgument(
        'camera_name', 
        default_value='front_camera',
        description='Name used for the node namespace. Should match the camera_name inside YAML.'
    )

    # ==========================================
    # 3. 定义要启动的独立 Node
    # ==========================================
    # 动态拼接命名空间，例如: /driver/hikvision/front_camera
    namespace = ['/driver/hikvision/', LaunchConfiguration('camera_name')]

    hik_camera_node = Node(
        package='hikvision_driver',
        executable='hikvision_driver_node',
        name='hikvision_driver_node',
        namespace=namespace,
        output='screen',
        # 【核心修改】直接把全局参数文件路径喂给 parameters
        parameters=[LaunchConfiguration('config_file')]
    )

    # ==========================================
    # 4. 返回 LaunchDescription
    # ==========================================
    return LaunchDescription([
        config_file_arg,
        camera_name_arg,
        hik_camera_node
    ])