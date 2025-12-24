cmake_minimum_required(VERSION 3.13)
project(rm_referee)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_definitions(-std=c++17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rm_referee_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

add_subdirectory(libs)
file(GLOB_RECURSE SOURCES "src/*.cc")

# 创建组件库
add_library(referee_node_component SHARED ${SOURCES})
target_link_libraries(referee_node_component ${PROJECT_NAME}_third_party ${Boost_LIBRARIES})
target_include_directories(referee_node_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Boost_INCLUDE_DIRS})

ament_target_dependencies(referee_node_component
  rclcpp
  rclcpp_components
  rm_referee_msgs
)

# 注册为组件
rclcpp_components_register_node(referee_node_component
  PLUGIN "RefereeNode"
  EXECUTABLE referee_node
)

# 安装库和可执行文件
install(TARGETS referee_node_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
