cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(itergrated)

find_package(PCL 1.3 REQUIRED)

message(STATUS "PCL_INCLUDE_DIRS = ${PCL_INCLUDE_DIRS}")
message(STATUS "PCL_LIBRARY_DIRS = ${PCL_LIBRARY_DIRS}")
message(STATUS "PCL_LIBRARIES    = ${PCL_LIBRARIES}")
message(STATUS "PCL_FOUND        = ${PCL_FOUND}")

include_directories(include)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (itergrated obstacle_detection_improve.cpp)
target_link_libraries (itergrated ${PCL_LIBRARIES})