cmake_minimum_required(VERSION 3.13)
project(rm_referee_mock)

set(CMAKE_EXPORT_COMPILE_COMMANDS true)
if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rm_referee_msgs REQUIRED)
find_package(SDL REQUIRED)

# keyboard_publisher_component
add_library(keyboard_publisher_component SHARED
  src/keyboard_publisher_node.cc
)

ament_target_dependencies(keyboard_publisher_component
  rclcpp
  rclcpp_components
  std_msgs
  rm_referee_msgs
)

target_include_directories(keyboard_publisher_component
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${SDL_INCLUDE_DIR}
)

target_link_libraries(keyboard_publisher_component
  SDL
)

# Register component
rclcpp_components_register_node(keyboard_publisher_component
  PLUGIN "rm_referee_mock::KeyboardPublisher"
  EXECUTABLE keyboard_publisher_node
)

# Install library and executable
install(TARGETS
  keyboard_publisher_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
