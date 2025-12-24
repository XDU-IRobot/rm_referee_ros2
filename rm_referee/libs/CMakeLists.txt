cmake_minimum_required(VERSION 3.5)

add_subdirectory(librm)

add_library(${PROJECT_NAME}_third_party INTERFACE)
target_link_libraries(${PROJECT_NAME}_third_party INTERFACE rm)