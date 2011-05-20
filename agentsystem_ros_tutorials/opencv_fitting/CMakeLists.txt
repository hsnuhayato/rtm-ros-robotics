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

# CMake settings for Debian package installation
set(CPACK_COMPONENTS_ALL fitting-base fitting-devel)
install(TARGETS fitting DESTINATION lib COMPONENT fitting-base)
install(TARGETS fitting_node DESTINATION bin COMPONENT fitting-base)
install(FILES include/fitting.h DESTINATION include/opencv_fitting COMPONENT fitting-devel)

set(CPACK_DEBIAN_PACKAGE_NAME opencvfitting)
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "my opencv fitting package")
set(CPACK_PACKAGE_VERSION "0.1")
set(CPACK_PACKAGE_INSTALL_DIRECTORY "opencv_fitting")
set(CPACK_PACKAGE_CONTACT "temp name <temp@temp.com>")

#set(CPACK_DEBIAN_DISTRIBUTION_RELEASES karmic lucid maverick natty)
#set(DPUT_HOST "ppa:user/package")
#set(CPACK_DEBIAN_DISTRIBUTION_NAME ubuntu)
#set(CPACK_DEBIAN_BUILD_DEPENDS_UBUNTU debhelper cmake
#                                      ros-diamondback-common-msgs
#                                      ros-diamondback-vision-opencv)
#set(CPACK_DEBIAN_PACKAGE_DEPENDS ${CPACK_COMPONENTS_ALL})
#set(CPACK_DEBIAN_PACKAGE_PRIORITY optional)
#set(CPACK_DEBIAN_PACKAGE_SECTION devel)
#set(CPACK_DEBIAN_PACKAGE_SOURCE_COPY svn export --force)
#set(CPACK_DEBIAN_PACKAGE_NOSIGN 1) # do not sign!

#set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake-modules")
include(CPack)
#include(DebSourcePPA)
