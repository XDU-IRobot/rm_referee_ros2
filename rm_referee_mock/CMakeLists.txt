cmake_minimum_required(VERSION 3.13)
project(rm_referee_mock)

set(CMAKE_EXPORT_COMPILE_COMMANDS true)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

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

rclcpp_components_register_node(dart_client_component
  PLUGIN "rm_referee_mock::DartClient"
  EXECUTABLE dart_client
)

#######################
########## rqt plugins
#######################
install(DIRECTORY assets
  DESTINATION share/${PROJECT_NAME}
)

# Install plugin descriptor for rqt
install(FILES
  plugin.xml
  DESTINATION share/${PROJECT_NAME}
)


ament_python_install_package(${PROJECT_NAME}) # Install Python module
ament_auto_package()
