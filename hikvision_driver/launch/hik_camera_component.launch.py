import os
import yaml  # 导入 Python 自带的 yaml 解析库
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode

def launch_setup(context, *args, **kwargs):
    # 1. 将 LaunchConfiguration 转换成真实的字符串文件路径
    config_file_path = LaunchConfiguration('config_file').perform(context)

    # 2. 用 Python 打开并解析这个 YAML 文件
    with open(config_file_path, 'r') as f:
        yaml_data = yaml.safe_load(f)

    # 3. 提取 camera_name (假设你的 yaml 结构是 /** -> ros__parameters -> camera_name)
    try:
        camera_name = yaml_data['/**']['ros__parameters']['camera_name']
    except KeyError:
        print(f"[ERROR] 无法在 {config_file_path} 中找到 camera_name! 将使用备用名 fallback_camera")
        camera_name = 'fallback_camera'

    # 4. 动态构建命名空间 (完全听命于 YAML)
    namespace = ['/driver/hikvision/', camera_name]

    # 5. 定义组件
    load_driver_component = LoadComposableNodes(
        target_container=LaunchConfiguration('container_name'),
        composable_node_descriptions=[
            ComposableNode(
                package='hikvision_driver',
                plugin='hikvision_driver::HikvisionDriver',
                name='hikvision_driver_node',
                namespace=namespace, 
                # 【终极黑盒】：只传文件，不再强行覆盖任何参数！
                parameters=[config_file_path], 
                extra_arguments=[{'use_intra_process_comms': True}]
            )
        ]
    )
    
    return [load_driver_component]

def generate_launch_description():
    pkg_share_dir = get_package_share_directory('hikvision_driver')
    default_config_path = os.path.join(pkg_share_dir, 'config', 'camera_params.yaml')

    # 现在只需要这两个参数了，camera_name 被彻底消灭！
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_path,
        description='Path to the camera parameter yaml file.'
    )

    container_name_arg = DeclareLaunchArgument(
        'container_name', default_value='camera_container'
    )

    # OpaqueFunction 允许我们在启动时先执行上面的 launch_setup 函数去读取文件
    return LaunchDescription([
        config_file_arg,
        container_name_arg,
        OpaqueFunction(function=launch_setup)
    ])