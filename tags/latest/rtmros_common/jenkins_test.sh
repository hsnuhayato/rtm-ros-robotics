#!/bin/bash -x

function error {
    rosrun rosunit clean_junit_xml.py
    echo "source $ROS_WORKSPACE/setup.bash"
    exit 1
}
wget 'http://svn.code.sf.net/p/jsk-ros-pkg/code/trunk/jsk.rosbuild?format=raw' -O /tmp/jsk.rosbuild.$$
. /tmp/jsk.rosbuild.$$ $1 -e

rm -fr $ROS_HOME/test_results/
trap error ERR

install-pkg 'http://svn.code.sf.net/p/jsk-ros-pkg/code/trunk/jsk.rosinstall?format=raw' http://rtm-ros-robotics.googlecode.com/svn/trunk/agentsystem_ros_tutorials/rtm-ros-robotics.rosinstall

PKGS='openrtm openrtm_ros_bridge euscollada openhrp3 hrpsys_tutorials hironx_ros_bridge'

compile-pkg $PKGS

sudo /etc/init.d/omniorb4-nameserver stop
yes | rosrun openrtm rtm-naming

test-pkg $PKGS

#
(which rosrun && rosrun rosunit clean_junit_xml.py; echo "done")     # check error
