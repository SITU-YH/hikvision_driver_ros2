既然你的 `hikvision_interface` 和 `hikvision_driver` 放在同一个代码仓库里，你需要一个能够从全局角度概括整个工程的 `README.md`。

这份 README 放在你仓库的根目录（即包含这两个包文件夹的那一层）是最合适的。

---

# Hikvision Camera ROS2 Suite

本仓库包含了一套完整的海康威视（Hikvision）工业相机 ROS2 驱动解决方案，采用了模块化设计，将数据接口与驱动实现分离。



### 包说明：

1. **`hikvision_interface`**: 定义了自定义的消息接口（如 `HikImageInfo`），用于向 ROS2 网络发布相机的原始数据及硬件状态。
2. **`hikvision_driver`**: 基于 C++ 实现的海康相机驱动，负责设备枚举、图像采集、格式转换以及动态参数调节。

---

## 环境准备

* **系统**: Ubuntu 22.04 (ROS2 Humble) 或更高版本。
* **依赖**: 请确保已安装海康 MVS Linux SDK（通常位于 `/opt/MVS`）。

---

## 构建说明

由于两个包在同一个仓库中，建议直接在你的 ROS2 工作空间（Workspace）的 `src` 目录下克隆本仓库，然后一并构建：

```bash
cd ~/your_ws/src
# 将本仓库代码放入此处，或者直接 clone
# git clone <your_repo_url>

cd ~/your_ws
# 同时编译两个包
colcon build --packages-select hikvision_interface hikvision_driver --symlink-install
source install/setup.bash

```

---

## 快速使用

### 1. 独立运行（用于单相机调试）

```bash
ros2 launch hikvision_driver hik_camera.launch.py camera_name:=argus_camera

```

### 2. 组件模式（用于复杂机器人系统）

为了降低 CPU 开销，建议将驱动作为组件加载：

```bash
# 启动容器
ros2 run rclcpp_components component_container --ros-args -r __node:=camera_container

# 加载相机驱动
ros2 launch hikvision_driver hik_camera_component.launch.py container_name:=camera_container 

```

---

## 配置指南

* **硬件参数**: 修改 `hikvision_driver/config/camera_params.yaml` 来配置曝光、增益等。
* **标定文件**: 在 `hikvision_driver/config/camera_info.yaml` 中填入你的相机标定矩。

---

## 常见问题排查 (Troubleshooting)

1. **RViz 无图像**:
* 请在 RViz 中打开 `Image` 面板，将 **Reliability Policy** 设置为 **`Best Effort`**（驱动采用此模式以确保实时性）。










