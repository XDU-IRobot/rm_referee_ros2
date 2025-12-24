cmake_minimum_required(VERSION 3.5)
project(rm_referee_msgs)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

file(GLOB_RECURSE msg_files msg/*.msg)
file(GLOB_RECURSE srv_files srv/*.srv)

# convert to relative paths
foreach(file ${msg_files} ${srv_files})
  file(RELATIVE_PATH rel_file ${CMAKE_CURRENT_SOURCE_DIR} ${file})
  set(rel_files ${rel_files} ${rel_file})
endforeach()

rosidl_generate_interfaces(${PROJECT_NAME}
  ${rel_files}
  DEPENDENCIES std_msgs
)

ament_package()