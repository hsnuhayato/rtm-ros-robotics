# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(hrpsys_ros_bridge)

# call catkin depends
find_package(catkin REQUIRED COMPONENTS rtmbuild roscpp sensor_msgs robot_state_publisher actionlib control_msgs tf camera_info_manager image_transport dynamic_reconfigure ) # pr2_controllers_msgs robot_monitor

# include rtmbuild
#include(${rtmbuild_PREFIX}/share/rtmbuild/cmake/rtmbuild.cmake)
if(EXISTS ${rtmbuild_SOURCE_DIR}/cmake/rtmbuild.cmake)
  include(${rtmbuild_SOURCE_DIR}/cmake/rtmbuild.cmake)
elseif(EXISTS ${CMAKE_INSTALL_PREFIX}/share/rtmbuild/cmake/rtmbuild.cmake)
  include(${CMAKE_INSTALL_PREFIX}/share/rtmbuild/cmake/rtmbuild.cmake)
else()
  get_cmake_property(_variableNames VARIABLES)
  foreach (_variableName ${_variableNames})
    message(STATUS "${_variableName}=${${_variableName}}")
  endforeach()
endif()
# include compile_robot_model.cmake
include(${PROJECT_SOURCE_DIR}/cmake/compile_robot_model.cmake)

# copy idl files from hrpsys
file(MAKE_DIRECTORY ${PROJECT_SOURCE_DIR}/idl)
find_package(PkgConfig)
pkg_check_modules(hrpsys hrpsys-base REQUIRED)
set(hrpsys_IDL_DIR ${hrpsys_PREFIX}/share/hrpsys/share/hrpsys/idl/)
if(EXISTS ${hrpsys_IDL_DIR})
  file(COPY
    ${hrpsys_IDL_DIR}
    DESTINATION ${PROJECT_SOURCE_DIR}/idl)
else()
  get_cmake_property(_variableNames VARIABLES)
  foreach (_variableName ${_variableNames})
    message(STATUS "${_variableName}=${${_variableName}}")
  endforeach()
  message(FATAL_ERROR "${hrpsys_IDL_DIR} is not found")
endif()

# define add_message_files before rtmbuild_init
add_message_files(FILES MotorStates.msg)

# initialize rtmbuild
rtmbuild_init()

# call catkin_package, after rtmbuild_init, before rtmbuild_gen*
catkin_package(
    DEPENDS hrpsys # TODO
    CATKIN-DEPENDS rtmbuild roscpp sensor_msgs robot_state_publisher actionlib control_msgs tf camera_info_manager image_transport dynamic_reconfigure # pr2_controllers_msgs robot_monitor
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

# generate idl
rtmbuild_genidl()

# generate bridge
rtmbuild_genbridge()

##
## hrpsys ros bridge tools
##
# pr2_controller_msgs is not catkinized
execute_process(
  COMMAND svn co --non-interactive --trust-server-cert https://code.ros.org/svn/wg-ros-pkg/stacks/pr2_controllers/tags/groovy/pr2_controllers_msgs /tmp/pr2_controllers_msgs
  OUTPUT_VARIABLE _download_output
  RESULT_VARIABLE _download_failed)
message("download pr2_controllers_msgs files ${_download_output}")
if (_download_failed)
  message(FATAL_ERROR "Download pr2_controllers_msgs failed : ${_download_failed}")
endif(_download_failed)
execute_process(
  COMMAND sh -c "ROS_PACKAGE_PATH=/tmp/pr2_controllers_msgs:$ROS_PACKAGE_PATH make -C /tmp/pr2_controllers_msgs"
  OUTPUT_VARIABLE _compile_output
  RESULT_VARIABLE _compile_failed)
message("Compile pr2_controllers_msgs files ${_compile_output}")
if (_compile_failed)
  message(FATAL_ERROR "Compile pr2_controllers_msgs failed : ${_compile_failed}")
endif(_compile_failed)


#include_directories(/opt/ros/$ENV{ROS_DISTRO}/stacks/pr2_controllers/pr2_controllers_msgs/msg_gen/cpp/include)
include_directories(/tmp/pr2_controllers_msgs/msg_gen/cpp/include)

rtmbuild_add_executable(HrpsysSeqStateROSBridge src/HrpsysSeqStateROSBridgeImpl.cpp src/HrpsysSeqStateROSBridge.cpp src/HrpsysSeqStateROSBridgeComp.cpp)
rtmbuild_add_executable(ImageSensorROSBridge src/ImageSensorROSBridge.cpp src/ImageSensorROSBridgeComp.cpp)
rtmbuild_add_executable(HrpsysJointTrajectoryBridge src/HrpsysJointTrajectoryBridge.cpp src/HrpsysJointTrajectoryBridgeComp.cpp)

if(TARGET compile_hrpsys)
  add_dependencies(HrpsysSeqStateROSBridge compile_hrpsys)
  add_dependencies(ImageSensorROSBridge compile_hrpsys)
  add_dependencies(HrpsysJointTrajectoryBridge compile_hrpsys)
endif()

install(PROGRAMS scripts/rtmlaunch scripts/rtmtest scripts/rtmstart.py
  DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN ".svn" EXCLUDE)
install(DIRECTORY scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  PATTERN ".svn" EXCLUDE
  PATTERN "rtmlaunch" EXCLUDE)


##
## test
##

