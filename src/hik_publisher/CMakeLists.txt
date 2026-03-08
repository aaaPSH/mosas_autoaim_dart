cmake_minimum_required(VERSION 3.8)
project(hik_publisher)
# Use C++ 17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/HikCameraNode.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  /opt/MVS/include
  include
)

# target_link_directories(${PROJECT_NAME} PUBLIC
#   /opt/MVS/lib/64
# )
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
  target_link_directories(${PROJECT_NAME} PUBLIC
    /opt/MVS/lib/aarch64
  )
else()
  target_link_directories(${PROJECT_NAME} PUBLIC
    /opt/MVS/lib/64
  )
endif()



target_link_libraries(${PROJECT_NAME}
  FormatConversion
  MediaProcess
  MvCameraControl
  MVRender
  MvUsb3vTL
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
  PLUGIN HikCameraNode
  EXECUTABLE ${PROJECT_NAME}_node
)

ament_auto_package(
  INSTALL_TO_SHARE
)
