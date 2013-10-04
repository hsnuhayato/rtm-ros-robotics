// http://www.ros.org/wiki/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"

#include <math.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "publish_screenpoint");

  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<geometry_msgs::PointStamped>("/image_painted/screenpoint", 1000);

  ros::Rate loop_rate(10);
  double i = 0;
  while (ros::ok())
    {
      geometry_msgs::PointStamped msg;
      msg.header.stamp = ros::Time().now();
      msg.point.x = 320 + 180*sin(i);
      msg.point.y = 240 + 180*cos(i);
      i += 0.1;
      ROS_INFO("%5.1f %5.1f", msg.point.x, msg.point.y);
      pub.publish(msg);
      ros::spinOnce();
      loop_rate.sleep();
    }

  return 0;
}
