cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(ground_filtering)

find_package(PCL 1.3 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (cluster cluster.cpp)
target_link_libraries (cluster ${PCL_LIBRARIES})