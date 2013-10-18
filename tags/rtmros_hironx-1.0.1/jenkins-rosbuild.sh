#!/bin/bash -x

trap 'exit 1' ERR

function setup {
    rm -fr rosbuild_ws
    mkdir -p rosbuild_ws
    cd rosbuild_ws
    unset ROS_PACKAGE_PATH
    rosws init
    rosws merge /opt/ros/groovy/
    rosws merge https://rtm-ros-robotics.googlecode.com/svn/trunk/rtm-ros-robotics.rosinstall
    rosws update
    cd ..
}

source /opt/ros/groovy/setup.bash
setup
cd rosbuild_ws
source setup.bash
rosmake --status-rate=0 --profile -V hironx_ros_bridge

source `rospack find openrtm_tools`/scripts/rtshell-setup.sh
trap ERR # remove trap
rtmtest hironx_ros_bridge hironx-test.launch
rtmtest hironx_ros_bridge hironx-ros-bridge-test.launch
rosrun rosunit clean_junit_xml.py
