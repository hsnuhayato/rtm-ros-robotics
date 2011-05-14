cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Debug)#RelWithDebInfo)

rosbuild_init()
add_definitions(-Wall)
rosbuild_add_boost_directories()

rosbuild_genmsg()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
rosbuild_add_library(fitting src/fitting.cpp)

rosbuild_add_executable(fitting_node src/fitting_node.cpp)
target_link_libraries(fitting_node fitting)

add_subdirectory(test)
