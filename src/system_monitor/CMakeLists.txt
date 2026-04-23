cmake_minimum_required(VERSION 3.8)
project(system_monitor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(autoaim_interfaces REQUIRED)

# 头文件目录
include_directories(include)

# 创建库
add_library(system_monitor SHARED
  src/SystemMonitorNode.cpp
)

# 链接库
ament_target_dependencies(system_monitor
  rclcpp
  rclcpp_components
  std_msgs
  autoaim_interfaces
)

# 注册为组件
rclcpp_components_register_node(system_monitor
  PLUGIN "system_monitor::SystemMonitorNode"
  EXECUTABLE system_monitor_node
)

# 安装
install(
  TARGETS system_monitor
  EXPORT export_system_monitor
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
