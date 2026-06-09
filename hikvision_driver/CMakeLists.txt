cmake_minimum_required(VERSION 3.8)
project(hikvision_driver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-Wall)

# find ros dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# find hikvision sdk
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(MVS REQUIRED)

# --- 修改 1: 声明头文件安装规范 ---
# 明确告诉 CMake 你的头文件在 include/hikvision_driver 中
include_directories(include)

# build components
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/hikvision_driver.cpp
)

# 确保链接属性正确
target_include_directories(${PROJECT_NAME} PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(${PROJECT_NAME} PRIVATE ${MVS_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME} ${MVS_LIBRARIES})

# register components
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "hikvision_driver::HikvisionDriver"
  EXECUTABLE ${PROJECT_NAME}_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# --- 修改 2: 启用 Scoped 模式并优化安装 ---
ament_auto_package(
  INSTALL_TO_SHARE launch config
  USE_SCOPED_HEADER_INSTALL_DIR
)
