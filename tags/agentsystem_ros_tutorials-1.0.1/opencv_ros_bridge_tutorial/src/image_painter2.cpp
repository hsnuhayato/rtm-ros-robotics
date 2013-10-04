// this code is based on http://www.ros.org/wiki/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/PointStamped.h>

namespace enc = sensor_msgs::image_encodings;

static const char WINDOW[] = "Image window";

class ImagePainter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Subscriber screen_sub_;

  int screen_x, screen_y;
public:
  ImagePainter()
    : it_(nh_)
  {
    image_pub_ = it_.advertise("image_painted", 1);
    image_sub_ = it_.subscribe("image", 1, &ImagePainter::imageCb, this);

    screen_x = 30; screen_y = 50;
    screen_sub_ = nh_.subscribe("/image_painted/screenpoint", 1, &ImagePainter::screenCb, this);

    cv::namedWindow(WINDOW);
  }

  ~ImagePainter()
  {
    cv::destroyWindow(WINDOW);
  }

  void screenCb(const geometry_msgs::PointStampedPtr& msg)
  {
    ROS_INFO("screen cb %3d %3d", (int)msg->point.x, (int)msg->point.y);
    screen_x = msg->point.x; screen_y = msg->point.y;
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
      {
	cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
      }
    catch (cv_bridge::Exception& e)
      {
	ROS_ERROR("cv_bridge exception: %s", e.what());
	return;
      }

    cv::circle(cv_ptr->image, cv::Point(screen_x, screen_y), 20, CV_RGB(0,0,255), 3);

    cv::imshow(WINDOW, cv_ptr->image);
    cv::waitKey(3);

    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_painter");
  ImagePainter ic;
  ros::spin();
  return 0;
}
