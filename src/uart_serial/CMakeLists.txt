cmake_minimum_required(VERSION 3.8)
project(uart_serial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ================= [ 1. 寻找系统依赖 ] =================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED) # 用于注册组件节点
find_package(std_msgs REQUIRED)          # 加上缺失的 std_msgs
find_package(autoaim_interfaces REQUIRED)
find_package(serial REQUIRED)

# ================= [ 2. 包含头文件目录 ] =================
include_directories(include)

# ================= [ 3. 编译共享库 (Component) ] =================
# 这里必须把您的源文件写进去！
add_library(${PROJECT_NAME}_node SHARED
  src/UartSerialNode.cpp
)

# 强制使用 C++17 标准
target_compile_features(${PROJECT_NAME}_node PUBLIC cxx_std_17)

# 给您的节点挂载刚才找到的那些依赖包
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  rclcpp_components
  std_msgs
  autoaim_interfaces
  serial
)

# ================= [ 4. 注册节点 & 生成可执行文件 ] =================
# 让 ROS 2 认识您的组件，并生成一个可以直接 ros2 run 的可执行文件
rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "uart_serial::UartSerialNode"
  EXECUTABLE ${PROJECT_NAME}_exec
)

# ================= [ 5. 安装规则 (没有这个 ros2 找不到它) ] =================
install(TARGETS
  ${PROJECT_NAME}_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# ================= [ 6. 自动测试模块 (保持您的原样) ] =================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()