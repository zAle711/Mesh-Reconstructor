cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(multiple_cloud_test)
find_package(PCL 1.3 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(multiple_cloud_test multiple_cloud.cpp)
target_link_libraries(multiple_cloud_test ${PCL_LIBRARIES})
