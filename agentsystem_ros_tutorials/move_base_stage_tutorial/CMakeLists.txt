if(NOT USE_ROSBUILD)
  cmake_minimum_required(VERSION 2.8.3)

  project(move_base_stage_tutorial)

  find_package(catkin REQUIRED COMPONENTS)

  catkin_package()

  install(DIRECTORY scripts
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    USE_SOURCE_PERMISSIONS
    PATTERN ".svn" EXCLUDE)

  install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    USE_SOURCE_PERMISSIONS
    PATTERN ".svn" EXCLUDE)

  install(DIRECTORY world
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    USE_SOURCE_PERMISSIONS
    PATTERN ".svn" EXCLUDE)

else()
  cmake_minimum_required(VERSION 2.4.6)

  include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

  rosbuild_init()

  #set the default path for built executables to the "bin" directory
  set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
  #set the default path for built libraries to the "lib" directory
  set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
endif()

