cmake_minimum_required(VERSION 3.13)
project(rm_referee_mock)

set(CMAKE_EXPORT_COMPILE_COMMANDS true)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

#######################
########## keyboard_publisher
#######################
ament_auto_add_library(keyboard_publisher_component SHARED
  src/keyboard_publisher.cc
)

target_include_directories(keyboard_publisher_component
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE
  ${SDL_INCLUDE_DIR}
)

# Register component
rclcpp_components_register_node(keyboard_publisher_component
  PLUGIN "rm_referee_mock::KeyboardPublisher"
  EXECUTABLE keyboard_publisher
)

#######################
########## dart_client
#######################
ament_auto_add_library(dart_client_component SHARED
  src/dart_client.cc
)

target_include_directories(dart_client_component
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Register component
rclcpp_components_register_node(dart_client_component
  PLUGIN "rm_referee_mock::DartClient"
  EXECUTABLE dart_client
)

# Install library and executable
install(TARGETS
  keyboard_publisher_component
  dart_client_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
