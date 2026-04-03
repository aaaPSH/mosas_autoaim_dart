cmake_minimum_required(VERSION 3.8)
project(save_frame)
# Use C++ 17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# 对于ament_auto_find_build_dependencies可能找不到的包，显式查找
find_package(yaml-cpp REQUIRED)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/SaveFrameNode.cpp
)

# 添加yaml-cpp的链接，因为ament_auto_find_build_dependencies可能找不到它
target_link_libraries(${PROJECT_NAME}
  yaml-cpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_cpplint
    ament_cmake_uncrustify
  )
  ament_lint_auto_find_test_dependencies()
endif()

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN SaveFrameNode
  EXECUTABLE ${PROJECT_NAME}_node
)

ament_auto_package(
  INSTALL_TO_SHARE
)
