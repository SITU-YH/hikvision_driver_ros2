import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    pkg_share_dir = get_package_share_directory('hikvision_driver')
    # 默认指向包内的配置文件
    default_config_path = os.path.join(pkg_share_dir, 'config', 'camera_params.yaml')

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_path,
        description='Path to the camera parameter yaml file.'
    )

    container_name_arg = DeclareLaunchArgument(
        'container_name', default_value='camera_container'
    )

    camera_name_arg = DeclareLaunchArgument(
        'camera_name', 
        default_value='front_camera',
        description='Name used for the node namespace.'
    )

    # 动态构建命名空间
    namespace = ['/driver/hikvision/', LaunchConfiguration('camera_name')]

    load_driver_component = LoadComposableNodes(
        target_container=LaunchConfiguration('container_name'),
        composable_node_descriptions=[
            ComposableNode(
                package='hikvision_driver',
                plugin='hikvision_driver::HikvisionDriver',
                name='hikvision_driver_node',
                namespace=namespace, 
                parameters=[
                    LaunchConfiguration('config_file'), 
                    {'camera_name': LaunchConfiguration('camera_name')} 
                ],
                extra_arguments=[{'use_intra_process_comms': True}]
            )
        ]
    )

    return LaunchDescription([
        config_file_arg,
        container_name_arg,
        camera_name_arg,
        load_driver_component
    ])