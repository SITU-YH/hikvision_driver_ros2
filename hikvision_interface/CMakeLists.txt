cmake_minimum_required(VERSION 3.8)
project(hikvision_interface)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED) # <--- 新增：显式寻找依赖包
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HikImageInfo.msg"
  DEPENDENCIES std_msgs builtin_interfaces
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# <--- 修改：把 builtin_interfaces 也导出
ament_export_dependencies(std_msgs builtin_interfaces rosidl_default_runtime)
ament_package()