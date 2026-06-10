import os
import yaml  # 导入 YAML 解析库
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    # 1. 将 LaunchConfiguration 转换成真实的字符串文件路径
    config_file_path = LaunchConfiguration('config_file').perform(context)

    # 2. 用 Python 打开并解析这个 YAML 文件
    with open(config_file_path, 'r') as f:
        yaml_data = yaml.safe_load(f)

    # 3. 提前从 YAML 提取 camera_name
    try:
        camera_name = yaml_data['/**']['ros__parameters']['camera_name']
    except KeyError:
        print(f"[ERROR] 无法在 {config_file_path} 中找到 camera_name! 将使用 fallback_camera")
        camera_name = 'fallback_camera'

    # 4. 动态构建命名空间 (完全听命于 YAML 里的名字)
    namespace = ['/driver/hikvision/', camera_name]

    # 5. 定义要启动的独立 Node
    hik_camera_node = Node(
        package='hikvision_driver',
        executable='hikvision_driver_node',
        name='hikvision_driver_node',
        namespace=namespace, 
        output='screen',
        # 【终极黑盒】：把文件路径直接喂给节点，不再做任何参数注入
        parameters=[config_file_path]
    )
    
    return [hik_camera_node]


def generate_launch_description():
    pkg_share_dir = get_package_share_directory('hikvision_driver')
    default_config_path = os.path.join(pkg_share_dir, 'config', 'camera_params.yaml')

    # 现在 Launch 只需要这一个参数！camera_name 参数已被彻底删除。
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_path,
        description='Absolute path to the camera parameter YAML file.'
    )

    # 使用 OpaqueFunction 在节点启动前执行文件读取逻辑
    return LaunchDescription([
        config_file_arg,
        OpaqueFunction(function=launch_setup)
    ])