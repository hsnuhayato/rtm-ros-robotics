#!/usr/bin/env python

# http://www.ros.org/wiki/ROS/Tutorials/WritingPublisherSubscriber%28python%29
import roslib; roslib.load_manifest('opencv_ros_bridge_tutorial')
import rospy;
import math;
from geometry_msgs.msg import PointStamped

pub = rospy.Publisher('/image_painted/screenpoint', PointStamped)
rospy.init_node('publish_screenpoint')
msg = PointStamped()
i = 0;
while not rospy.is_shutdown():
    msg.header.stamp = rospy.Time.now()
    msg.point.x = 320 + 180*math.sin(i)
    msg.point.y = 240 + 180*math.cos(i)
    i += 0.1
    print msg;
    pub.publish(msg)
    rospy.sleep(0.1)


