import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    pkg_share_dir = get_package_share_directory('hikvision_driver')
    default_config_path = os.path.join(pkg_share_dir, 'config', 'camera_params.yaml')

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_path,
        description='Path to the camera parameter yaml file.'
    )

    container_name_arg = DeclareLaunchArgument(
        'container_name', default_value='camera_container'
    )

    # 【新增】加上 camera_name 参数，为了跟 yaml 和独立节点保持行为一致
    camera_name_arg = DeclareLaunchArgument(
        'camera_name', 
        default_value='front_camera',
        description='Name used for the node namespace. Should match the camera_name inside YAML.'
    )

    # 【新增】动态拼接命名空间，防止多台相机挂载在同一容器时发生重名冲突
    namespace = ['/driver/hikvision/', LaunchConfiguration('camera_name')]

    load_driver_component = LoadComposableNodes(
        target_container=LaunchConfiguration('container_name'),
        composable_node_descriptions=[
            ComposableNode(
                package='hikvision_driver',
                plugin='hikvision_driver::HikvisionDriver',
                name='hikvision_driver_node',
                namespace=namespace,  # <--- 使用拼接好的命名空间
                parameters=[
                    LaunchConfiguration('config_file'), # 还是加载 YAML，但不用非得有 camera_name
                    {'camera_name': LaunchConfiguration('camera_name')} # 【核心修改】强制注入/覆盖
                ],
                extra_arguments=[{'use_intra_process_comms': True}]
            )
        ]
    )

    return LaunchDescription([
        config_file_arg,
        container_name_arg,
        camera_name_arg,  # <--- 不要忘记在这里返回
        load_driver_component
    ])